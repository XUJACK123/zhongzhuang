#!/usr/bin/env python3
"""Wall-bounded CUDA PPO, CPU MuJoCo, isolated real-data candidate contract."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

import mujoco
import numpy as np
import torch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from fifty_mujoco.envs.sample_env31_realdata import RealDataHardwareEnv31, configuration
from fifty_mujoco.envs.sample_env31 import rpy_from_wxyz
from fifty_mujoco.learning.sample_ppo import SamplePolicy31, SamplePPO
from fifty_mujoco.learning.checkpoint import save_checkpoint
from fifty_mujoco.transfer.sample_source import verified_artifact


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def equilibrium_action(env):
    env.reset(610000)
    env.data.qvel[:] = 0
    mujoco.mj_forward(env.model, env.data)
    contact = np.zeros(env.model.nv)
    com = env.data.subtree_com[env.base].copy()
    yr, yl = (env.data.geom_xpos[env.model.geom(n).id, 1]
              for n in ("right_wheel_collision_0", "left_wheel_collision_0"))
    left_fraction = float(np.clip((com[1]-yr)/(yl-yr), 0., 1.))
    for name, fraction in (("right_wheel_collision_0", 1-left_fraction), ("left_wheel_collision_0", left_fraction)):
        jac, jacr = np.zeros((3, env.model.nv)), np.zeros((3, env.model.nv))
        mujoco.mj_jacGeom(env.model, env.data, jac, jacr, env.model.geom(name).id)
        contact += jac.T @ [0., 0., fraction*env.model.body_mass.sum()*9.81]
    passive = env._spring_torque(env.source.defaults, np.zeros(6))[0]
    needed = env.data.qfrc_bias[env.source.qvel_ids] - contact[env.source.qvel_ids] - passive
    action = needed / np.array([20., 30., 1., 20., 30., 1.])
    action[[2, 5]] = 0
    return np.clip(action, -1., 1.)


def teacher(observation, base, gains=None):
    o = observation
    gains = gains or {"pitch_kp":2.,"pitch_kd":.5,"roll_kp":3.,"roll_kd":.4,"velocity":.5}
    pitch, roll = torch.asin(o[:, 3].clamp(-.99, .99)), -torch.asin(o[:, 4].clamp(-.99, .99))
    action = base.expand(len(o), -1).clone()
    action[:, 0] -= 2*pitch
    action[:, 3] += 2*pitch
    correction = gains["roll_kp"]*roll + gains["roll_kd"]*(4*o[:, 0])
    action[:, 1] += correction
    action[:, 4] += correction
    forward = gains["pitch_kp"]*pitch + gains["pitch_kd"]*(4*o[:, 1]) + gains["velocity"]*(o[:, 29]/2)
    turn = -.15*(4*o[:, 2]) + .15*o[:, 27]
    action[:, 2], action[:, 5] = forward-turn, forward+turn
    bound = o.new_tensor([1, 1, 2, 1, 1, 2])
    return torch.maximum(torch.minimum(action, bound), -bound)


def initialize(args, env):
    policy = SamplePolicy31(verified_artifact("model_3050.pt"), args.device)
    base = equilibrium_action(env)
    if args.init:
        saved = torch.load(args.init, map_location=args.device, weights_only=True)
        policy.load_state_dict(saved["model"])
    if args.teacher_init:
        # Distil explicit state feedback, then adapt it with on-policy PPO.
        # Samples are synthetic state features, never counted as physical trials.
        optimizer = torch.optim.Adam(policy.actor.parameters(), lr=1e-3)
        origin = torch.tensor(env.reset(610000), device=args.device)
        scale = origin.new_tensor([.1]*3 + [.08,.06,.03] + [0,0,0] + [.12]*4 + [.1]*6 + [.3]*6 + [.2]*6)
        for _ in range(700):
            obs = origin[None] + torch.randn((512,31), device=args.device)*scale
            target = teacher(obs, origin.new_tensor(base),env.candidate.get("teacher_feedback"))
            loss = (policy.actor(obs)-target).square().mean()
            optimizer.zero_grad(); loss.backward(); optimizer.step()
        with torch.no_grad():
            policy.log_std.copy_(origin.new_tensor([.08,.08,.05,.08,.08,.05]).log())
    return policy, base


def reward(env, info, action, previous):
    dt = env.source.policy_dt
    rp = np.radians(info["roll_pitch_deg"])
    velocity = info["point_velocity_xy"][0]
    height_error = info["clearance_m"] - env.candidate["standing_clearance_m"]
    q = env.data.qpos[env.source.qpos_ids]
    leg_error = q[[0,1,3,4]] - env.source.defaults[[0,1,3,4]]
    saturation = sum(info["saturation_counts"])/(6*max(1, info["controls"]))
    return float(dt*(2*math.exp(-np.dot(rp,rp)/.05**2)
                     + math.exp(-(velocity/.1)**2) + 2*math.exp(-(height_error/.02)**2)
                     + math.exp(-np.dot(leg_error,leg_error)/.15**2)
                     - .2*saturation - .01*np.square(np.clip(action,-2,2)-previous).sum())
                 - 5*bool(info["failure"]))


def evaluate(policy, count=4, seconds=15., seed_base=610000):
    rows = []
    for j in range(count):
        env = RealDataHardwareEnv31(episode_seconds=seconds)
        obs = env.reset(seed_base+j)
        sample = []
        for _ in range(env.max_steps):
            with torch.no_grad():
                a = policy.actor(torch.tensor(obs, device=next(policy.parameters()).device)[None])[0].cpu().numpy()
            obs, _, done, timeout, info = env.step(a)
            if info["time_s"] > 2:
                sample.append([abs(info["clearance_m"]-env.candidate["standing_clearance_m"]),
                               *np.abs(info["roll_pitch_deg"]), abs(info["point_velocity_xy"][0]),
                               float(np.linalg.norm(env.data.qpos[:2]))])
            if done or timeout:
                break
        value = np.asarray(sample) if sample else np.full((1,5),1e3)
        metrics = {"clearance_mae_m": float(value[:,0].mean()), "tilt_p95_deg": float(np.percentile(value[:,1:3],95)),
                   "vx_mae_m_s": float(value[:,3].mean()), "displacement_max_m": float(value[:,4].max())}
        passed = bool(timeout and not done and metrics["clearance_mae_m"] < .02 and metrics["tilt_p95_deg"] < 8
                      and metrics["vx_mae_m_s"] < .08 and metrics["displacement_max_m"] < .3)
        rows.append({"seed": seed_base+j, "seconds": info["time_s"], "failure": info["failure"], "passed": passed, **metrics})
    score = sum((0 if r["passed"] else 100) + 10*r["clearance_mae_m"] + .05*r["tilt_p95_deg"]
                + r["vx_mae_m_s"] + r["displacement_max_m"] for r in rows)/len(rows)
    return {"rows": rows, "passed": sum(r["passed"] for r in rows), "trials": len(rows), "score": score}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--init", type=Path)
    p.add_argument("--teacher-init", action="store_true")
    p.add_argument("--device", default="cuda")
    p.add_argument("--seconds", type=float, default=1200)
    p.add_argument("--updates", type=int, default=3000)
    p.add_argument("--envs", type=int, default=16)
    p.add_argument("--steps", type=int, default=64)
    p.add_argument("--actor-lr", type=float, default=3e-5)
    p.add_argument("--eval-every", type=int, default=100)
    p.add_argument("--save-every", type=int, default=100)
    p.add_argument("--episode-seconds", type=float, default=12)
    p.add_argument("--seed", type=int, default=510000)
    args = p.parse_args()
    torch.set_num_threads(1); torch.manual_seed(args.seed)
    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    args.out.mkdir(parents=True, exist_ok=False)
    envs = [RealDataHardwareEnv31(episode_seconds=args.episode_seconds) for _ in range(args.envs)]
    policy, base = initialize(args, envs[0])
    optimizer = SamplePPO(policy, actor_lr=args.actor_lr, critic_lr=3e-4)
    serial = 0
    def reset(env):
        nonlocal serial
        seed = args.seed+serial; serial += 1
        if seed >= 600000:
            raise RuntimeError("training seed reservation exhausted")
        return env.reset(seed)
    observations = [reset(e) for e in envs]
    contract = {"configuration_sha256": digest(PROJECT/"configs/realdata_20261117.json"),
                "trainer_sha256": digest(__file__), "environment_sha256": digest(PROJECT/"src/fifty_mujoco/envs/sample_env31_realdata.py"),
                "environment": envs[0].snapshot(), "device": args.device,
                "gpu_name": torch.cuda.get_device_name() if args.device.startswith("cuda") else None,
                "task": "new_standing_contract_balance", "seed_range": [args.seed,599999],
                "initial_checkpoint": str(args.init) if args.init else None,
                "initial_checkpoint_sha256": digest(args.init) if args.init else None,
                "teacher_initialization": args.teacher_init, "static_teacher_action": base.tolist()}
    (args.out/"contract.json").write_text(json.dumps(contract,indent=2))
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    start, update, transitions, best = time.monotonic(), 0, 0, math.inf
    def save(name):
        save_checkpoint(args.out/name, {"schema":"realdata_balance31_ppo_v1", "model":policy.state_dict(),
                         "actor_optimizer":optimizer.actor_optimizer.state_dict(), "critic_optimizer":optimizer.critic_optimizer.state_dict(),
                         "torch_rng":torch.get_rng_state(), "cuda_rng":torch.cuda.get_rng_state_all() if args.device.startswith('cuda') else [],
                         "update":update, "transitions":transitions, "episodes_started":serial, "contract":contract})
    names = ("observation","action","log_prob","value","reward","next_value","terminated","truncated","mean","std")
    save("update_000000.pt")
    initial = evaluate(policy, count=2, seconds=10)
    (args.out/"initial_evaluation.json").write_text(json.dumps(initial,indent=2))
    print(json.dumps({"initial_evaluation":initial}),flush=True)
    try:
        with (args.out/"rows.jsonl").open("a") as log:
            for update in range(1,args.updates+1):
                if stopped or time.monotonic()-start >= args.seconds:
                    break
                rollout = {n:[] for n in names}; outcomes=[]
                for _ in range(args.steps):
                    obs=torch.tensor(np.stack(observations),device=args.device)
                    with torch.no_grad():
                        dist=policy.distribution(obs); action=dist.sample(); values=policy.value(obs); lp=dist.log_prob(action).sum(-1)
                    results=[]
                    for env,a in zip(envs,action.cpu().numpy()):
                        previous=np.asarray(env.last_action).copy();o,_,t,d,info=env.step(a)
                        results.append((o,reward(env,info,a,previous),t,d,info))
                    terminal=torch.tensor(np.stack([r[0] for r in results]),device=args.device)
                    with torch.no_grad():nv=policy.value(terminal)
                    rewards=torch.tensor([r[1] for r in results],device=args.device)
                    terminated=torch.tensor([r[2] for r in results],device=args.device)
                    truncated=torch.tensor([r[3] for r in results],device=args.device)
                    for name,value in zip(names,(obs,action,lp,values,rewards,nv,terminated,truncated,dist.mean,dist.stddev)):
                        rollout[name].append(value.detach())
                    observations=[]
                    for env,result in zip(envs,results):
                        if result[2] or result[3]:
                            outcomes.append({"time_s":result[4]["time_s"],"failure":result[4]["failure"]})
                            observations.append(reset(env))
                        else:observations.append(result[0])
                metrics=optimizer.update({k:torch.stack(v) for k,v in rollout.items()},warmup=update<=5,epochs=4,batch_size=256)
                transitions+=args.envs*args.steps
                row={"update":update,"transitions":transitions,"wall_s":time.monotonic()-start,
                     "reward_mean":float(torch.stack(rollout["reward"]).mean()),"outcomes":outcomes,**metrics}
                if update%args.eval_every==0:
                    evaluation=evaluate(policy, count=4,seconds=15)
                    row["evaluation"]=evaluation
                    if evaluation["score"]<best:
                        best=evaluation["score"];save("best.pt")
                log.write(json.dumps(row,allow_nan=False)+'\n');log.flush()
                print(json.dumps({k:v for k,v in row.items() if k!='outcomes'},allow_nan=False),flush=True)
                if update%args.save_every==0:save(f"update_{update:06d}.pt")
    finally:
        save("last.pt")
        (args.out/"training_report.json").write_text(json.dumps({"update":update,"transitions":transitions,
           "wall_s":time.monotonic()-start,"stopped_by_signal":stopped,"device":args.device,"best_development_score":best if math.isfinite(best) else None},indent=2))


if __name__ == "__main__":
    main()

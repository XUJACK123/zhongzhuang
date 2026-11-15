#!/usr/bin/env python3
"""Get-up PPO entry point with lazy runtime imports and an explicit plant adapter.

--prepare-only checks provenance without importing Torch or a physics engine.
The factory callable accepts plan/profile/frozen_balance_actor/device and returns
an environment exposing reset(seed, parameters), step(action), protocol_schema,
and autonomous_balance_validation=True. On entering BALANCE it advances the
frozen controller internally until the 10 s outcome. Each returned transition
therefore corresponds to a GETUP/HANDOFF action from the learner.
"""

import argparse
import copy
import hashlib
import importlib
import json
import math
from pathlib import Path
import signal
import sys
import time

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
sys.path.insert(0, str(PROJECT / "tools"))
from fifty_mujoco.contracts.getup31 import SCHEMA
from fifty_mujoco.contracts.sample_actor31 import finite_vector
from fifty_mujoco.learning.getup_curriculum import Curriculum, plan_digest, resumed_curriculum
from sample_hardware_prepare_next import PLAN, sha256, validate_plan


def preparation():
    plan = json.loads((PROJECT / PLAN).read_text(encoding="utf-8"))
    profile = json.loads((PROJECT / plan["baseline"]["profile"]).read_text(encoding="utf-8"))
    validate_plan(plan, profile)
    checkpoint = PROJECT / plan["baseline"]["checkpoint"]
    if sha256(checkpoint) != plan["baseline"]["sha256"]:
        raise ValueError("frozen balance checkpoint changed")
    return plan, profile, checkpoint


def initialize_policy(checkpoint, device, torch):
    from fifty_mujoco.learning.sample_ppo import SamplePolicy31
    from fifty_mujoco.transfer.sample_source import verified_artifact
    policy = SamplePolicy31(verified_artifact("model_3050.pt"), device)
    saved = torch.load(checkpoint, map_location=device, weights_only=True)["model"]
    actor_state = {name.removeprefix("actor."): value for name, value in saved.items() if name.startswith("actor.")}
    policy.actor.load_state_dict(actor_state, strict=True)
    with torch.no_grad():
        policy.log_std.copy_(saved["log_std"])
    if not all(torch.isfinite(value).all() for value in policy.state_dict().values()):
        raise ValueError("non-finite initialized policy")
    frozen_balance = copy.deepcopy(policy.actor).requires_grad_(False).eval()
    # The freshly constructed critic and PPO optimizers belong to this task.
    return policy, frozen_balance


def validate_transition(result):
    observation, reward, terminated, truncated, info = result
    observation = finite_vector(observation, 31, "terminal observation")
    if not isinstance(reward, (int, float)) or not math.isfinite(reward):
        raise ValueError("finite scalar reward required")
    if type(terminated) is not bool or type(truncated) is not bool or terminated and truncated:
        raise ValueError("invalid terminal flags")
    if info["policy_phase_before"] not in ("GETUP", "HANDOFF"):
        raise ValueError("learner action recorded outside GETUP/HANDOFF")
    if terminated or truncated:
        if type(info["success"]) is not bool:
            raise ValueError("terminal success flag required")
        if type(info["balance_validation_completed"]) is not bool:
            raise ValueError("balance validation completion must be boolean")
        if info["success"] and (not terminated or not info["balance_validation_completed"] or info["failure"] is not None):
            raise ValueError("success requires completed autonomous balance validation")
        if not info["success"] and not info["failure"]:
            raise ValueError("failed episode requires first failure reason")
    return observation, reward, terminated, truncated, info


def train(args, plan, profile, checkpoint):
    import torch
    from fifty_mujoco.learning.sample_ppo import SamplePPO
    from fifty_mujoco.learning.checkpoint import save_checkpoint
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    module_name, function_name = args.factory.split(":", 1)
    module = importlib.import_module(module_name)
    factory = getattr(module, function_name)
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    policy, balance = initialize_policy(checkpoint, args.device, torch)
    optimizer = SamplePPO(policy, actor_lr=args.actor_lr, critic_lr=args.critic_lr)
    state = json.loads(args.curriculum_state.read_text(encoding="utf-8")) if args.curriculum_state else None
    curriculum = Curriculum(plan, state)
    contract = {"schema": SCHEMA, "plan_sha256": plan_digest(plan),
                "balance_checkpoint_sha256": sha256(checkpoint), "trainer_sha256": sha256(Path(__file__)),
                "factory": args.factory, "factory_sha256": sha256(Path(module.__file__)),
                "ppo_sha256": sha256(PROJECT / "src/fifty_mujoco/learning/sample_ppo.py"),
                "hyperparameters": {name: getattr(args, name) for name in ("envs", "steps", "epochs", "batch_size", "actor_lr", "critic_lr")}}
    code_files = ("contracts/getup31.py", "contracts/sample_actor31.py", "learning/getup_curriculum.py",
                  "envs/sample_env31.py", "envs/sample_env31_hardware.py", "envs/sample_env31_hardware_fast.py",
                  "envs/sample_env31_hardware_guarded.py", "transfer/sample_source.py")
    contract["code_sha256"] = {name: sha256(PROJECT / "src/fifty_mujoco" / name) for name in code_files}
    contract["profile_sha256"] = sha256(PROJECT / plan["baseline"]["profile"])
    episodes_started, update = 0, 0
    if args.resume:
        saved = torch.load(args.resume, map_location=args.device, weights_only=True)
        if saved["contract"] != contract:
            raise ValueError("resume code, stage or configuration changed")
        policy.load_state_dict(saved["model"], strict=True)
        optimizer.actor_optimizer.load_state_dict(saved["actor_optimizer"])
        optimizer.critic_optimizer.load_state_dict(saved["critic_optimizer"])
        curriculum = resumed_curriculum(plan, saved["curriculum"], state)
        torch.set_rng_state(saved["torch_rng"].cpu())
        if args.device == "cuda" and saved.get("cuda_rng"):
            torch.cuda.set_rng_state_all([value.cpu() for value in saved["cuda_rng"]])
        episodes_started, update = saved["episodes_started"], saved["update"]
    envs = [factory(plan=plan, profile=profile, frozen_balance_actor=balance, device=args.device) for _ in range(args.envs)]
    for env in envs:
        if env.protocol_schema != SCHEMA or env.autonomous_balance_validation is not True:
            raise ValueError("environment does not implement the get-up transition contract")
    reset_events = []
    def reset(env):
        nonlocal episodes_started
        seed = plan["evaluation"]["seed_ranges_inclusive"]["train"][0] + episodes_started
        parameters = curriculum.episode_parameters(seed)
        episodes_started += 1
        reset_events.append(parameters)
        return finite_vector(env.reset(seed=seed, parameters=parameters), 31, "reset observation")
    observations = [reset(env) for env in envs]
    args.out.mkdir(parents=True, exist_ok=False)
    stopped = False
    def request_stop(signum, frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    deadline = time.monotonic() + args.max_seconds
    def save():
        path = args.out / f"update_{update:06d}.pt"
        save_checkpoint(path, {"schema": SCHEMA, "contract": contract, "model": policy.state_dict(),
                               "actor_optimizer": optimizer.actor_optimizer.state_dict(),
                               "critic_optimizer": optimizer.critic_optimizer.state_dict(),
                               "curriculum": curriculum.state_dict(), "episodes_started": episodes_started,
                               "pending_reset_events": list(reset_events),
                               "update": update, "torch_rng": torch.get_rng_state(),
                               "cuda_rng": torch.cuda.get_rng_state_all() if args.device == "cuda" else [],
                               "resume_scope": "optimizer_and_rng_restored_plants_reset_with_new_seeds"})
    save()
    names = ("observation", "action", "log_prob", "value", "reward", "next_value", "terminated", "truncated", "mean", "std")
    try:
        with (args.out / "rows.jsonl").open("a", encoding="utf-8") as history:
            for _ in range(args.updates):
                collected = {name: [] for name in names}
                outcomes = []
                for _ in range(args.steps):
                    if stopped or time.monotonic() >= deadline:
                        break
                    obs = torch.tensor(observations, dtype=torch.float32, device=args.device)
                    with torch.no_grad():
                        distribution = policy.distribution(obs)
                        actions = distribution.sample()
                        values = policy.value(obs)
                        log_prob = distribution.log_prob(actions).sum(-1)
                    results = [validate_transition(env.step(action)) for env, action in zip(envs, actions.cpu().tolist())]
                    terminal = torch.tensor([r[0] for r in results], dtype=torch.float32, device=args.device)
                    with torch.no_grad():
                        next_values = policy.value(terminal)
                    rewards = torch.tensor([r[1] for r in results], dtype=torch.float32, device=args.device)
                    if not torch.isfinite(rewards).all():
                        raise ValueError("non-finite rewards")
                    terminated = torch.tensor([r[2] for r in results], device=args.device)
                    truncated = torch.tensor([r[3] for r in results], device=args.device)
                    for name, value in zip(names, (obs, actions, log_prob, values, rewards, next_values,
                                                   terminated, truncated, distribution.mean, distribution.stddev)):
                        collected[name].append(value.detach())
                    observations = []
                    for env, result in zip(envs, results):
                        if result[2] or result[3]:
                            outcomes.append({key: result[4][key] for key in ("success", "failure", "balance_validation_completed", "reset_class", "episode_parameters")})
                            observations.append(reset(env))
                        else:
                            observations.append(result[0])
                if stopped or time.monotonic() >= deadline or not collected["reward"]:
                    break
                metrics = optimizer.update({name: torch.stack(values) for name, values in collected.items()},
                                           epochs=args.epochs, batch_size=args.batch_size)
                update += 1
                row = {"update": update, "stage": curriculum.stage, "outcomes": outcomes,
                       "reset_events": list(reset_events), **metrics}
                history.write(json.dumps(row, allow_nan=False) + "\n")
                history.flush()
                reset_events.clear()
                save()
                print(json.dumps(row, allow_nan=False), flush=True)
    finally:
        save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--factory", help="physics adapter module:callable")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--curriculum-state", type=Path)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--max-seconds", type=float, default=3600.)
    parser.add_argument("--updates", type=int, default=100)
    parser.add_argument("--envs", type=int, default=8)
    parser.add_argument("--steps", type=int, default=128)
    parser.add_argument("--epochs", type=int, default=2)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--actor-lr", type=float, default=1e-6)
    parser.add_argument("--critic-lr", type=float, default=3e-4)
    parser.add_argument("--seed", type=int, default=100000)
    args = parser.parse_args()
    plan, profile, checkpoint = preparation()
    if args.prepare_only:
        print(json.dumps({"status": "provenance_ready_runtime_validation_pending", "schema": SCHEMA,
                          "plan_sha256": plan_digest(plan), "balance_checkpoint_sha256": sha256(checkpoint),
                          "actor_initialization": "balance_actor_and_log_std_only_fresh_critic_and_optimizers"}, indent=2))
        return
    if not args.factory or ":" not in args.factory or args.out is None:
        parser.error("training requires --factory module:callable and --out")
    numeric = (args.max_seconds, args.actor_lr, args.critic_lr)
    if min(args.envs, args.steps, args.epochs, args.batch_size, args.updates) <= 0 or not all(math.isfinite(v) and v > 0 for v in numeric):
        parser.error("positive training sizes and time budget required")
    train(args, plan, profile, checkpoint)


if __name__ == "__main__":
    main()

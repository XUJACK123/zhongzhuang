#!/usr/bin/env python3
"""Record development-only dynamic feasibility; never promote a curriculum."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np
import torch

from sample_getup_train import PROJECT, preparation, initialize_policy
from fifty_mujoco.envs.sample_env31_getup import make_env
from fifty_mujoco.learning.getup_curriculum import Curriculum, plan_digest


def snapshot(env):
    e = env.env
    return {"time_s": float(e.data.time), "qpos": e.data.qpos.tolist(),
            "qvel": e.data.qvel.tolist(), "info": e.last_info,
            "motor_command_nm": getattr(e.fast_actuators, "command", np.zeros(6)).tolist(),
            "contacts": [{"geoms": [e.model.geom(int(i)).name for i in (c.geom1, c.geom2)],
                          "distance_m": float(c.dist)} for c in e.data.contact]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--stages", nargs="+", default=["near_stand", "supported_survey", "support_release", "disabled_rest"])
    parser.add_argument("--gains", nargs="+", type=float, default=[1., .8])
    args = parser.parse_args()
    if args.out.exists():
        parser.error("use a new output path")
    torch.set_num_threads(1)
    torch.manual_seed(100000)
    plan, profile, checkpoint = preparation()
    policy, balance = initialize_policy(checkpoint, "cpu", torch)
    env = make_env(plan, profile, balance)
    course = Curriculum(plan)
    rows, started = [], time.monotonic()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    for stage in args.stages:
        course.index = next(i for i, x in enumerate(course.stages) if x["name"] == stage)
        for gain in args.gains:
            for seed in range(200000, 200000 + args.seeds):
                parameters = course.episode_parameters(seed, split="development")
                parameters["motor_gain"] = [gain, gain, 1., gain, gain, 1.]
                parameters["nominal"] = gain == 1.
                row = {"stage": stage, "gain": gain, "seed": seed, "parameters": parameters,
                       "success": False, "trace": []}
                try:
                    observation = env.reset(seed, parameters)
                    row["initial"] = snapshot(env)
                    for step in range(600):
                        with torch.no_grad():
                            action = policy.actor(torch.tensor(observation)[None])[0].tolist()
                        observation, reward, terminated, truncated, info = env.step(action)
                        if step % 10 == 0 or terminated or truncated:
                            row["trace"].append({"state": asdict(env.previous_sample), "action": action,
                                                 "phase": env.decision.phase.value, "reward": reward,
                                                 "motor_command_nm": env.env.fast_actuators.command.tolist()})
                        if terminated or truncated:
                            row["success"], row["failure"] = info["success"], info["failure"]
                            break
                    else:
                        row["failure"] = "probe_step_limit"
                except Exception as exc:
                    row["failure"] = f"{type(exc).__name__}: {exc}"
                row["final"] = snapshot(env)
                rows.append(row)
                report = {"schema": "getup31_dynamic_probe_v1", "evidence_kind": "simulation",
                          "plan_sha256": plan_digest(plan), "controller": "frozen_balance_actor",
                          "balance_checkpoint_sha256": plan["baseline"]["sha256"],
                          "elapsed_s": time.monotonic() - started, "rows": rows}
                args.out.write_text(json.dumps(report, indent=2, allow_nan=False))
                print(json.dumps({k: row[k] for k in ("stage", "gain", "seed", "success", "failure")}), flush=True)
    print(json.dumps(dict(Counter(row["failure"] for row in rows))), flush=True)


if __name__ == "__main__":
    main()

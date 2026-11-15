#!/usr/bin/env python3
"""Prepare or execute get-up development/heldout trials using the frozen protocol."""

import argparse
import importlib
import json
import math
from pathlib import Path
import sys
import time

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
sys.path.insert(0, str(PROJECT / "tools"))
from fifty_mujoco.contracts.getup31 import SCHEMA
from fifty_mujoco.learning.getup_curriculum import Curriculum, acceptance, plan_digest
from fifty_mujoco.transfer.getup_trace import HARD_FAILURES
from sample_getup_train import initialize_policy, preparation, validate_transition
from sample_getup_offline import write_json
from sample_hardware_prepare_next import sha256


def baseline_regression(path, plan):
    if path is None:
        return {"passed": False, "report": None}
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    candidate = data["candidate"]
    development = data["checks"]["development"]
    control = candidate["deployment_control"]
    passed = (candidate["sha256"] == plan["baseline"]["sha256"]
              and control["wheel_kd"] == plan["baseline"]["wheel_kd"]
              and control["target_knee_margin_rad"] == plan["baseline"]["target_knee_margin_rad"]
              and development["primary_trials"] >= 200 and development["strict_all_trials_passed"]
              and development["extra_protocols"] >= 3 and development["extra_passed"] == development["extra_protocols"])
    return {"passed": bool(passed), "report": str(path), "sha256": sha256(path),
            "provenance": "saved_frozen_balance_reconstruction_report"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--split", choices=("development", "heldout_nominal", "heldout_combined"), default="development")
    parser.add_argument("--stage", choices=("near_stand", "supported_survey", "support_release", "disabled_rest", "joint_randomization"))
    parser.add_argument("--balance-regression-report", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--max-seconds", type=float, default=3600.)
    args = parser.parse_args()
    plan, profile, balance_path = preparation()
    low, high = plan["evaluation"]["seed_ranges_inclusive"][args.split]
    if args.prepare_only:
        print(json.dumps({"schema": SCHEMA, "split": args.split, "seeds": [low, high], "trials": high - low + 1,
                          "plan_sha256": plan_digest(plan), "physics_validation_completed": False}, indent=2))
        return
    if args.checkpoint is None or args.out is None or not math.isfinite(args.max_seconds) or args.max_seconds <= 0:
        parser.error("evaluation requires --checkpoint, --out and a positive time budget")
    import torch
    torch.set_num_threads(1)
    policy, balance = initialize_policy(balance_path, args.device, torch)
    checkpoint = torch.load(args.checkpoint, map_location=args.device, weights_only=True)
    if checkpoint["schema"] != SCHEMA or checkpoint["contract"]["plan_sha256"] != plan_digest(plan):
        raise ValueError("get-up checkpoint protocol mismatch")
    policy.load_state_dict(checkpoint["model"], strict=True)
    policy.eval()
    course = Curriculum(plan, checkpoint["curriculum"])
    if args.stage:
        course.index = next(i for i, stage in enumerate(course.stages) if stage["name"] == args.stage)
    if args.split != "development" and course.stage != "joint_randomization":
        parser.error("final heldout evaluation uses the complete joint_randomization task")
    module_name, factory_name = plan["getup"]["physics_adapter"].split(":", 1)
    factory = getattr(importlib.import_module(module_name), factory_name)
    env = factory(plan=plan, profile=profile, frozen_balance_actor=balance, device=args.device)
    if args.out.exists():
        parser.error("use a new output path for each evaluation")
    regression = baseline_regression(args.balance_regression_report, plan)
    rows, started = [], time.monotonic()
    checkpoint_sha = sha256(args.checkpoint)
    for seed in range(low, high + 1):
        if time.monotonic() - started >= args.max_seconds:
            break
        parameters = course.episode_parameters(seed, split=args.split)
        observation = env.reset(seed=seed, parameters=parameters)
        while True:
            with torch.no_grad():
                action = policy.actor(torch.tensor(observation, dtype=torch.float32, device=args.device)[None])[0].cpu().tolist()
            observation, reward, terminated, truncated, info = validate_transition(env.step(action))
            if terminated or truncated:
                break
        row = {"seed": seed, "reset_class": info["reset_class"], "success": info["success"],
               "failure": info["failure"], "hard_limit_failure": info["failure"] in HARD_FAILURES,
               "balance_validation_completed": info["balance_validation_completed"], "evidence_kind": "simulation",
               "plan_sha256": plan_digest(plan), "getup_checkpoint_sha256": checkpoint_sha,
               "balance_checkpoint_sha256": plan["baseline"]["sha256"], "parameters": parameters}
        rows.append(row)
        write_json(args.out.with_suffix(".rows.json"), rows)
        print(json.dumps({key: row[key] for key in ("seed", "reset_class", "success", "failure")}), flush=True)
    report = {"schema": SCHEMA, "evaluation_id": f"{checkpoint_sha}:{args.split}:{course.stage}",
              "stage": course.stage, "split": args.split, "seeds": [r["seed"] for r in rows],
              "successes": sum(r["success"] for r in rows),
              "hard_limit_failures": sum(r["hard_limit_failure"] for r in rows),
              "nominal_balance_regression_passed": regression["passed"], "balance_regression": regression,
              "getup_checkpoint_sha256": checkpoint_sha, "plan_sha256": plan_digest(plan),
              "complete": len(rows) == high - low + 1, "evidence_kind": "simulation",
              "rows_file": str(args.out.with_suffix(".rows.json"))}
    if args.split != "development":
        report["acceptance"] = acceptance(rows, plan, args.split)
    write_json(args.out, report)


if __name__ == "__main__":
    main()

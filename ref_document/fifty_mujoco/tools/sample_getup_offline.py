#!/usr/bin/env python3
"""Prepare, test, replay and score the get-up protocol without a physics runtime."""

import argparse
import ast
import hashlib
import io
import json
from pathlib import Path
import sys
import unittest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from fifty_mujoco.contracts.getup31 import GetupController, SCHEMA
from fifty_mujoco.learning.getup_curriculum import Curriculum, acceptance, plan_digest
from fifty_mujoco.transfer.getup_trace import replay
from sample_hardware_prepare_next import PLAN, prepare


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    # Atomic replacement keeps a retry from consuming a partly written state.
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def guarded_knee_limits(plan, profile):
    result = [list(limits) for limits in profile["firmware_target_knee_limits_rad"]]
    margin = plan["baseline"]["target_knee_margin_rad"]
    result[0][0] += margin
    result[1][1] -= margin
    return result


def offline_checks(plan):
    prepare(PROJECT, PROJECT / "reports/sample_weight")
    GetupController(plan)
    loader = unittest.TestLoader()
    suite = loader.discover(str(PROJECT / "tests"), pattern="test_getup31_offline.py")
    suite.addTests(loader.discover(str(PROJECT / "tests"), pattern="test_sample_hardware_prepare_next.py"))
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=1).run(suite)
    sources = [PROJECT / PLAN, Path(__file__),
               PROJECT / "tools/sample_getup_train.py",
               PROJECT / "tools/sample_getup_evaluate.py",
               PROJECT / "src/fifty_mujoco/contracts/getup31.py",
               PROJECT / "src/fifty_mujoco/learning/getup_curriculum.py",
               PROJECT / "src/fifty_mujoco/transfer/getup_trace.py",
               PROJECT / "src/fifty_mujoco/envs/sample_env31_getup.py",
               PROJECT / "tests/test_getup31_offline.py"]
    for source in sources:
        if source.suffix == ".py":
            ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    curriculum = Curriculum(plan)
    examples = []
    for index in range(len(curriculum.stages)):
        curriculum.index = index
        examples.append(curriculum.episode_parameters(100000 + index))
    report = {"schema": SCHEMA, "status": "offline_checks_passed" if result.wasSuccessful() else "offline_checks_failed",
              "tests_run": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
              "test_output": stream.getvalue(), "plan_sha256": plan_digest(plan),
              "source_sha256": {str(p.relative_to(PROJECT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
              "curriculum_examples": examples, "examples_are_parameter_specs": True,
              "runtime_pending": ["CPU_MuJoCo_getup_adapter_dynamics_and_contact_geometry_validation",
                                  "CPU_Torch_actor_initialization_and_PPO_smoke",
                                  "dynamic_reachability_and_force_margin", "training_and_heldout_evaluation",
                                  "MCU_port_timing_and_supported_hardware_validation"],
              "physics_validation_completed": False}
    out = PROJECT / "reports/sample_weight/hardware_getup_offline_readiness_20261115.json"
    write_json(out, report)
    print(json.dumps({"status": report["status"], "tests": result.testsRun, "failures": len(result.failures),
                      "errors": len(result.errors), "report": str(out)}, indent=2))
    if not result.wasSuccessful():
        print(stream.getvalue())
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="Run standard-library tests and regenerate evidence reports")
    trace = sub.add_parser("replay", help="Check one complete JSONL trial")
    trace.add_argument("--input", type=Path, required=True)
    trace.add_argument("--out", type=Path, required=True)
    course = sub.add_parser("curriculum", help="Apply one development report to a saved curriculum state")
    course.add_argument("--report", type=Path, required=True)
    course.add_argument("--state", type=Path, required=True)
    score = sub.add_parser("acceptance", help="Score frozen, complete heldout replay summaries")
    score.add_argument("--input", type=Path, required=True)
    score.add_argument("--split", choices=("heldout_nominal", "heldout_combined"), required=True)
    score.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    plan = read_json(PROJECT / PLAN)
    if args.command == "check":
        offline_checks(plan)
    elif args.command == "replay":
        lines = [json.loads(line) for line in args.input.read_text(encoding="utf-8-sig").splitlines() if line.strip()]
        if not lines:
            parser.error("trace is empty")
        profile = read_json(PROJECT / plan["baseline"]["profile"])
        report = replay(lines[0], lines[1:], plan, guarded_knee_limits(plan, profile))
        write_json(args.out, report)
        print(json.dumps({key: report[key] for key in ("evidence_kind", "success", "failure", "samples")}))
    elif args.command == "curriculum":
        curriculum = Curriculum(plan, read_json(args.state) if args.state.exists() else None)
        promoted = curriculum.record(read_json(args.report))
        write_json(args.state, curriculum.state_dict())
        print(json.dumps({"stage": curriculum.stage, "promoted": promoted}))
    else:
        rows = read_json(args.input)
        if not isinstance(rows, list) or not rows:
            parser.error("input must contain replay summaries")
        kinds = {row["evidence_kind"] for row in rows}
        if len(kinds) != 1 or not kinds <= {"simulation", "hardware_synced"}:
            parser.error("heldout scoring requires one simulation or synchronized hardware evidence kind")
        models = {(row["getup_checkpoint_sha256"], row["balance_checkpoint_sha256"]) for row in rows}
        if len(models) != 1 or any(row["plan_sha256"] != plan_digest(plan) for row in rows):
            parser.error("checkpoint or protocol differs within heldout input")
        report = acceptance(rows, plan, args.split)
        report.update(evidence_kind=next(iter(kinds)), checkpoints=list(next(iter(models))))
        write_json(args.out, report)
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

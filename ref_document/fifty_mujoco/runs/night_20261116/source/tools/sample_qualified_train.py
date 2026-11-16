#!/usr/bin/env python3
"""Bounded continuation on the qualified-odometry balance task.

Reuses the established PPO/checkpoint runner with a separately fingerprinted
environment, development evaluator and optional per-episode joint randomization.
"""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

import sample_hardware_guarded_train as runner
from sample_qualified_evaluate import CLASSES, episode, parameters
from fifty_mujoco.envs.sample_env31_qualified import QualifiedHardwareEnv31, ENV_SCHEMA
from fifty_mujoco.transfer.firmware_runtime import source_fingerprint

DOMAIN = "combined" if "--combined" in sys.argv else "nominal"
if "--combined" in sys.argv:
    sys.argv.remove("--combined")


class TrainingEnvironment(QualifiedHardwareEnv31):
    def reset(self, seed=0, command=(0., 0., .1)):
        if hasattr(self, "fast_actuators") and DOMAIN == "combined":
            params = parameters(seed, DOMAIN)
            self.motor_gain[:] = params["motor_gain"]
            self.fast_actuators.gain[:] = self.motor_gain
            self.model.geom_friction[:, 0] = params["effective_friction"]
            self.spring_scale = params["spring_scale"]
            for spring in self.source.spring_config["springs"]:
                spring["reference_force_n"] = self.source.profile["spring_force_n"]*self.spring_scale
            self.randomized_parameters = params
            if EPISODE_LOG is not None:
                with EPISODE_LOG.open("a") as stream:
                    stream.write(json.dumps({"seed": seed, "command": list(command), "parameters": params})+"\n")
        return super().reset(seed, command)


original_fingerprint = runner.fingerprint
def fingerprint(env):
    result = original_fingerprint(env)
    result.update(environment=ENV_SCHEMA, firmware_source=source_fingerprint(), domain=DOMAIN,
                  startup_included=False, ideal_sensor_inputs=True)
    for name in ("tools/sample_qualified_train.py", "tools/sample_qualified_evaluate.py",
                 "src/fifty_mujoco/envs/sample_env31_qualified.py", "src/fifty_mujoco/transfer/firmware_runtime.py"):
        result["code"][name] = hashlib.sha256((runner.PROJECT/name).read_bytes()).hexdigest()
    result["randomization"] = {"nominal_fraction": .2, "dm_gain": [.8, 1.], "friction": [.6, .9],
                               "spring_force_scale": [.9, 1.1]} if DOMAIN == "combined" else None
    # Actual random episode values are logged; the contract records the distribution.
    if DOMAIN == "combined":
        result["motor_gain"] = "episode_randomized"
        result["spring_scale"] = "episode_randomized"
    return result


def evaluate(policy, args):
    rows, summaries = [], {}
    for i, name in enumerate(CLASSES):
        group = [episode(policy.actor, name, 2100000+i*1000+j, args.eval_seconds, DOMAIN)
                 for j in range(args.eval_seeds)]
        rows.extend(group)
        wins = sum(r["thresholds_met"] for r in group)
        summaries[name] = {"episodes": len(group), "rate": wins/len(group), "passed": wins}
        print(json.dumps({"eval_class": name, **summaries[name]}), flush=True)
    return {"rows": rows, "summaries": summaries,
            "all_classes_threshold_met": all(s["rate"] >= .9 for s in summaries.values())}


original_train = runner.train
def train(args):
    result = original_train(args)
    result["randomization_enabled"] = DOMAIN == "combined"
    result["scope"] = "Already balancing, current C odometry-quality/reference logic, ideal sensors; no startup or hardware acceptance."
    result["episode_parameters_log"] = str(EPISODE_LOG) if DOMAIN == "combined" else None
    (args.out/"training_report.json").write_text(json.dumps(result, indent=2)+"\n")
    return result


EPISODE_LOG = None
if __name__ == "__main__":
    out = Path(sys.argv[sys.argv.index("--out")+1])
    out.parent.mkdir(parents=True, exist_ok=True)
    EPISODE_LOG = out.with_suffix(".episodes.jsonl")
    if EPISODE_LOG.exists():
        raise ValueError("use a fresh output and episode log")
    runner.SampleEnv31 = TrainingEnvironment
    runner.ENV_SCHEMA = ENV_SCHEMA
    runner.fingerprint = fingerprint
    runner.evaluate = evaluate
    runner.train = train
    # Stay within the current firmware's commissioned command envelope.
    runner.TRAIN_COMMANDS = list(CLASSES.values()) + [[.15, .3, .1], [.15, -.3, .1], [-.15, .3, .1], [-.15, -.3, .1]]
    raise SystemExit(runner.main())

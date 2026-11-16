#!/usr/bin/env python3
"""Development/heldout evaluation of the current C odometry balance adapter."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
import multiprocessing
from pathlib import Path
import sys
import time
import numpy as np
import torch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/"src"))
from fifty_mujoco.envs.sample_env31_qualified import QualifiedHardwareEnv31, ENV_SCHEMA
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact
from fifty_mujoco.transfer.firmware_runtime import source_fingerprint
from fifty_mujoco.envs.sample_env31 import wilson_interval

CLASSES = {"stand": [0., 0., .1], "forward": [.15, 0., .1], "backward": [-.15, 0., .1],
           "left": [0., .3, .1], "right": [0., -.3, .1]}
ACTOR = None


def parameters(seed, domain):
    rng = np.random.default_rng(seed+71000000)
    gains = np.ones(6)
    friction, spring = .75, 1.
    if domain == "combined" and seed % 5:
        gains[[0, 1, 3, 4]] = rng.uniform(.8, 1., 4)
        friction, spring = float(rng.uniform(.6, .9)), float(rng.uniform(.9, 1.1))
    return {"motor_gain": gains.tolist(), "effective_friction": friction, "spring_scale": spring}


def episode(actor, name, seed, duration, domain="nominal", fixed_parameters=None):
    params = parameters(seed, domain) if fixed_parameters is None else fixed_parameters
    env = QualifiedHardwareEnv31(episode_seconds=duration, wheel_kd=.14, **params)
    command = CLASSES.get(name, [0., 0., .1])
    obs = env.reset(seed, command)
    samples, confidences = [], []
    saturation, controls, min_margin = np.zeros(6), 0, float("inf")
    protocol = json.loads((PROJECT/"configs/sample_command_evaluation.json").read_text())
    for tick in range(env.max_steps):
        if name == "reversal":
            command = [protocol["reversal_5s_segments"][min(7, int(tick*.01/5))], 0., .1]
            env.request(command)
        elif name == "turning":
            command = [*protocol["turning_5s_segments"][min(7, int(tick*.01/5))], .1]
            env.request(command)
        with torch.no_grad():
            obs_tensor = torch.tensor(obs, device=next(actor.parameters()).device)[None]
            action = actor(obs_tensor)[0].cpu().numpy()
        obs, _, done, truncated, info = env.step(action)
        saturation += info["saturation_counts"]
        controls += info["controls"]
        if info["spring_margins_m"]:
            min_margin = min(min_margin, *info["spring_margins_m"])
        confidences.append(info["odometry_confidence"])
        if info["time_s"] >= 2:
            # Score the requested motion even when firmware gates its command.
            samples.append([info["point_velocity_xy"][0]-command[0], info["yaw_rate"]-command[1],
                            abs(info["clearance_m"]-.1), *np.abs(info["roll_pitch_deg"])])
        if done or truncated:
            break
    metrics = {"samples": len(samples)}
    if samples:
        values = np.array(samples)
        metrics.update(point_vx_rmse_m_s=float(np.sqrt(np.mean(values[:, 0]**2))),
                       yaw_rate_rmse_rad_s=float(np.sqrt(np.mean(values[:, 1]**2))),
                       clearance_mae_m=float(values[:, 2].mean()),
                       roll_abs_p95_deg=float(np.percentile(values[:, 3], 95)),
                       pitch_abs_p95_deg=float(np.percentile(values[:, 4], 95)))
    limits = protocol["thresholds"]
    fractions = saturation/max(1, controls)
    passed = bool(truncated and not done and samples and
                  metrics["point_vx_rmse_m_s"] <= limits["vx_rmse_max_m_s"] and
                  metrics["yaw_rate_rmse_rad_s"] <= limits["wz_rmse_max_rad_s"] and
                  metrics["clearance_mae_m"] <= limits["clearance_mae_max_m"] and
                  max(metrics["roll_abs_p95_deg"], metrics["pitch_abs_p95_deg"]) <= limits["tilt_p95_max_deg"] and
                  fractions.max() <= limits["motor_saturation_max_per_motor"])
    return {"scenario": name, "seed": seed, "domain": domain, "parameters": params,
            "elapsed_s": info["time_s"], "completed": bool(truncated and not done),
            "first_failure": info["failure"], "thresholds_met": passed, "metrics": metrics,
            "motor_saturation_fractions": fractions.tolist(), "minimum_spring_margin_m": min_margin,
            "odometry_low_confidence_fraction": float(np.mean(np.array(confidences)<.25))}


def initialize(path):
    global ACTOR
    torch.set_num_threads(1)
    p = SamplePolicy31(verified_artifact("model_3050.pt"))
    p.load_state_dict(torch.load(path, map_location="cpu", weights_only=True)["model"])
    ACTOR = p.actor.eval()


def run(job):
    return episode(ACTOR, **job)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--domain", choices=("nominal", "combined"), default="nominal")
    parser.add_argument("--seeds-per-class", type=int, default=4)
    parser.add_argument("--seed-base", type=int, default=2000000)
    parser.add_argument("--duration", type=float, default=20.)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--extra-protocols", action="store_true")
    parser.add_argument("--deadline-epoch", type=float, required=True)
    args = parser.parse_args()
    if args.out.exists() or args.out.with_suffix(".rows.jsonl").exists():
        parser.error("use a fresh report path")
    if not 1 <= args.seeds_per_class <= 100 or not 1 <= args.workers <= 8 or not 2 < args.duration <= 120:
        parser.error("invalid evaluation sizes")
    jobs = [{"name": name, "seed": args.seed_base+i*1000+j, "duration": args.duration, "domain": args.domain}
            for i, name in enumerate(CLASSES) for j in range(args.seeds_per_class)]
    if args.extra_protocols:
        jobs += [{"name": name, "seed": args.seed_base+5000+i, "duration": args.duration, "domain": "nominal"}
                 for i, name in enumerate(("reversal", "turning"))]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    rows, started = [], time.monotonic()
    with args.out.with_suffix(".rows.jsonl").open("w") as stream, ProcessPoolExecutor(
        max_workers=args.workers, mp_context=multiprocessing.get_context("spawn"),
        initializer=initialize, initargs=(str(args.checkpoint),)) as pool:
        pending = {}
        iterator = iter(jobs)
        for _ in range(args.workers):
            job = next(iterator, None)
            if job is not None:
                pending[pool.submit(run, job)] = job
        while pending:
            future = next(as_completed(pending))
            pending.pop(future)
            row = future.result()
            rows.append(row)
            stream.write(json.dumps(row)+"\n")
            stream.flush()
            print(json.dumps({"done": len(rows), "total": len(jobs), "passed": sum(r["thresholds_met"] for r in rows),
                              "wall_s": time.monotonic()-started}), flush=True)
            if time.time() < args.deadline_epoch-120:
                job = next(iterator, None)
                if job is not None:
                    pending[pool.submit(run, job)] = job
    groups = {}
    for name in {row["scenario"] for row in rows}:
        group = [r for r in rows if r["scenario"] == name]
        wins = sum(r["thresholds_met"] for r in group)
        groups[name] = {"trials": len(group), "passed": wins, "wilson_95": wilson_interval(wins, len(group))}
    report = {"schema": ENV_SCHEMA, "checkpoint": str(args.checkpoint),
              "checkpoint_sha256": hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
              "firmware_sha256": source_fingerprint(), "domain": args.domain,
              "seed_base": args.seed_base, "duration_s": args.duration,
              "complete": len(rows) == len(jobs), "summaries": groups, "rows": rows,
              "wall_seconds": time.monotonic()-started,
              "scope": "Already balancing reset; ideal IMU/encoders with actual C odometry quality and reference logic; excludes startup, IMU filtering, CAN timing and hardware acceptance."}
    args.out.write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

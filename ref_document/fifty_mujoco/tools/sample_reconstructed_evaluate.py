#!/usr/bin/env python3
"""Command-point tracking evaluation of the frozen reconstructed profile.

Reports engineering thresholds/intervals independently of source provenance.
No result here accepts the target robot or restores the missing Isaac runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/"src"))
from fifty_mujoco.envs.sample_env31 import ENV_SCHEMA, SampleEnv31, wilson_interval
from fifty_mujoco.transfer.sample_checkpoint import load_sample_actor
from fifty_mujoco.transfer.sample_source import verified_artifact


def episode(actor, name, seed, duration):
    env = SampleEnv31(duration)
    commands = {"stand": (0, 0, .1), "forward": (.15, 0, .1),
        "backward": (-.15, 0, .1), "backward_fast_ood": (-.3, 0, .1),
        "left": (0, .3, .1), "right": (0, -.3, .1)}
    protocol = json.loads((PROJECT/"configs/sample_command_evaluation.json").read_text())
    obs = env.reset(seed, commands.get(name, (0, 0, .1)))
    rows = []
    saturation = np.zeros(6)
    controls = 0
    max_speed = np.zeros(6)
    min_spring_margin = math.inf
    stop_starts, stop_drifts, steady_stop_s = None, [], 0.
    for tick in range(env.max_steps):
        if name == "reversal":
            env.request([protocol["reversal_5s_segments"][min(7, int(tick*env.source.policy_dt/5))], 0, .1])
        elif name == "turning":
            env.request([*protocol["turning_5s_segments"][min(7, int(tick*env.source.policy_dt/5))], .1])
        with torch.no_grad():
            action = actor(torch.tensor(obs)[None])[0].numpy()
        obs, _, terminated, truncated, info = env.step(action)
        command = info["applied_command"]
        saturation += info["saturation_counts"]
        controls += info["controls"]
        max_speed = np.maximum(max_speed, info["max_motor_speed_rad_s"])
        if info["spring_margins_m"]:
            min_spring_margin = min(min_spring_margin, *info["spring_margins_m"])
        if abs(command[0])+abs(command[1]) < 1e-6:
            steady_stop_s += env.source.policy_dt
            if steady_stop_s >= .5:
                if stop_starts is None:
                    stop_starts = env._point().copy()
                stop_drifts.append(float(np.linalg.norm((env._point()-stop_starts)[:2])))
        else:
            steady_stop_s, stop_starts = 0., None
        if info["time_s"] >= 2:
            rows.append([info["point_velocity_xy"][0]-command[0],
                info["point_velocity_xy"][1], info["yaw_rate"]-command[1],
                abs(info["clearance_m"]-command[2]), *np.abs(info["roll_pitch_deg"]),
                np.linalg.norm(info["reference_error_xy_m"]), abs(info["reference_yaw_error_rad"])])
        if terminated or truncated:
            break
    samples = np.asarray(rows)
    completed = truncated and not terminated
    metrics = {"samples": len(rows)}
    if len(rows):
        metrics.update(point_vx_rmse_m_s=float(np.sqrt(np.mean(samples[:, 0]**2))),
            point_vy_rmse_m_s=float(np.sqrt(np.mean(samples[:, 1]**2))),
            yaw_rate_rmse_rad_s=float(np.sqrt(np.mean(samples[:, 2]**2))),
            clearance_mae_m=float(samples[:, 3].mean()), roll_abs_p95_deg=float(np.percentile(samples[:, 4], 95)),
            pitch_abs_p95_deg=float(np.percentile(samples[:, 5], 95)),
            reference_position_rmse_m=float(np.sqrt(np.mean(samples[:, 6]**2))),
            reference_heading_mae_rad=float(samples[:, 7].mean()))
    fractions = saturation/max(1, controls)
    threshold = protocol["thresholds"]
    quality = bool(completed and len(rows) and metrics["point_vx_rmse_m_s"] <= threshold["vx_rmse_max_m_s"]
        and metrics["yaw_rate_rmse_rad_s"] <= threshold["wz_rmse_max_rad_s"]
        and metrics["clearance_mae_m"] <= threshold["clearance_mae_max_m"]
        and max(metrics["roll_abs_p95_deg"], metrics["pitch_abs_p95_deg"]) <= threshold["tilt_p95_max_deg"]
        and fractions.max() <= threshold["motor_saturation_max_per_motor"])
    return {"scenario": name, "seed": seed, "duration_s": duration,
        "elapsed_s": float(env.data.time), "completed": completed, "first_failure": info["failure"],
        "thresholds_met": quality, "metrics": metrics, "motor_saturation_fractions": fractions.tolist(),
        "motor_max_speeds_rad_s": max_speed.tolist(),
        "minimum_spring_margin_m": min_spring_margin if math.isfinite(min_spring_margin) else None,
        "max_settled_stop_drift_m": max(stop_drifts) if stop_drifts else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seeds-per-class", type=int, default=5)
    parser.add_argument("--duration", type=float, default=40)
    args = parser.parse_args()
    if not 1 <= args.seeds_per_class <= 40 or not math.isfinite(args.duration) or args.duration < 2.01:
        parser.error("1..40 seeds and finite duration>2s required")
    torch.set_num_threads(1)
    actor, _ = load_sample_actor(verified_artifact("model_3050.pt"))
    rows, summaries = [], {}
    # Separate reconstructed-development seeds, never the heldout acceptance sets.
    for index, name in enumerate(("stand", "forward", "backward", "left", "right",
                                   "backward_fast_ood", "reversal", "turning")):
        count = args.seeds_per_class if index < 5 else 1
        current = [episode(actor, name, 71000+index*100+i, args.duration) for i in range(count)]
        rows.extend(current)
        successes = sum(r["thresholds_met"] for r in current)
        summaries[name] = {"trials": count, "completed": sum(r["completed"] for r in current),
            "thresholds_met": successes, "rate": successes/count, "wilson_95": wilson_interval(successes, count)}
        print(json.dumps({"scenario": name, **summaries[name]}), flush=True)
    report = {"tool": "sample-reconstructed-command-evaluation", "schema": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": "reconstructed_profile_only", "environment": ENV_SCHEMA,
        "model_xml_sha256": hashlib.sha256(SampleEnv31().source.xml.encode()).hexdigest(),
        "checkpoint_sha256": hashlib.sha256(verified_artifact("model_3050.pt").read_bytes()).hexdigest(),
        "implementation_sha256": {str(p.relative_to(PROJECT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [Path(__file__).resolve(), PROJECT/"src/fifty_mujoco/envs/sample_env31.py"]},
        "summaries": summaries, "rows": rows, "target_model_accepted": False,
        "original_PhysX_parity": False, "source_archive_complete": False,
        "formal_heldout_acceptance": False, "training_executed": False,
        "semantics": {"forward_metric": "command_point_displacement_in_current_heading_frame",
            "yaw_metric": "wrapped_heading_difference_over_actual_step_dt",
            "warmup_s": 2, "first_failure": True, "automatic_reset": False,
            "roll_pitch": "separate_absolute_p95", "seed_scope": "reconstructed_development_only"}}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")


if __name__ == "__main__":
    main()

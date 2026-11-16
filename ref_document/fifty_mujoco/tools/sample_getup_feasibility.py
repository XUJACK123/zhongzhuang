#!/usr/bin/env python3
"""Bounded startup-control and timestep diagnostics, with original limits intact."""
import itertools
import json
from pathlib import Path
import time
import numpy as np
import torch
from sample_getup_train import PROJECT, preparation, initialize_policy
from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31


def trial(pose, gain, ramp, action, physics_dt=.001):
    env = GuardedHardwareEnv31(episode_seconds=.5, reset_mode=pose, wheel_kd=.14,
                              motor_gain=[gain, gain, 1., gain, gain, 1.])
    env.model.opt.timestep = physics_dt
    env.source.physics_substeps = round(.002 / physics_dt)
    env.reset(200050)
    apply = env.fast_actuators.apply
    def scaled(requested, q, qd):
        result = apply(requested, q, qd)
        scale = 0. if ramp is None else min(1., float(env.data.time) / ramp) if ramp else 1.
        return result * scale
    env.fast_actuators.apply = scaled
    initial = env.data.qpos.copy()
    for _ in range(env.max_steps):
        _, _, done, timeout, info = env.step(action)
        if done or timeout:
            break
    return {"pose": pose, "gain": gain, "ramp_s": ramp, "action": list(action),
            "physics_dt_s": physics_dt, "time_s": info["time_s"], "failure": info["failure"],
            "initial_qpos": initial.tolist(), "final_qpos": env.data.qpos.tolist(),
            "final_qvel": env.data.qvel.tolist(), "spring_margin_m": info["spring_margins_m"]}


def main():
    torch.set_num_threads(1)
    started = time.monotonic()
    rows = []
    for pose, gain, ramp in itertools.product(("default", "natural"), (1., .8), (.2, .5, 0.)):
        for h, k, w in itertools.product((-1., 0., 1.), repeat=3):
            rows.append(trial(pose, gain, ramp, [h, k, w, -h, -k, w]))
    passive = [trial(pose, 1., None, [0.]*6, dt) for pose in ("default", "natural")
               for dt in (.001, .0005, .00025)]
    summaries = []
    for pose, gain, ramp in itertools.product(("default", "natural"), (1., .8), (.2, .5, 0.)):
        group = [r for r in rows if (r["pose"], r["gain"], r["ramp_s"]) == (pose, gain, ramp)]
        best = max(group, key=lambda r: r["time_s"])
        summaries.append({"pose": pose, "gain": gain, "ramp_s": ramp, "trials": len(group),
                          "survived_0_5s": sum(r["failure"] is None for r in group),
                          "longest_s": best["time_s"], "best_action": best["action"],
                          "best_failure": best["failure"]})
    out = PROJECT / "reports/sample_weight/getup_feasibility_20261116.json"
    if out.exists():
        raise ValueError("diagnostic output already exists")
    report = {"schema": "getup_feasibility_diagnostic_v1", "evidence_kind": "simulation",
              "elapsed_s": time.monotonic()-started, "summaries": summaries,
              "passive_timestep_convergence": passive, "rows": rows,
              "interpretation": "Fixed bounded action grid is not a proof of global impossibility. No limits, inertia or damping were relaxed. No-ramp cases diagnose the ramp only; they are not an approved startup controller."}
    out.write_text(json.dumps(report, indent=2))
    print(json.dumps({"summaries": summaries, "passive": [{k:r[k] for k in ("pose", "physics_dt_s", "time_s", "failure")} for r in passive]}, indent=2))


if __name__ == "__main__":
    main()

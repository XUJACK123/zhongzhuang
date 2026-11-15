#!/usr/bin/env python3
"""V2 source-only static and short bounded-PD diagnostics; never PPO."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np
import torch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from fifty_mujoco.contracts.sample_actor31 import JOINT_ORDER, bottom_clearance
from fifty_mujoco.transfer.sample_source import SampleSource


def probe():
    source = SampleSource()
    model, data = source.model, source.data
    initial_contacts = source.contacts()
    base = model.body("base_link").id
    clearance = bottom_clearance(data.xpos[base, 2], [0, 0, -1],
                                 source.env["env"]["clearance_box_center"],
                                 source.env["env"]["clearance_box_halfsize"])
    springs, lengths = source.spring_torque(source.defaults, np.zeros(6))
    ratios = [source.actuator_module.knee_ratio(sign * source.defaults[k] + math.pi/2, source.hardware)
              for k, sign in ((1, 1), (4, -1))]
    checks = {"mass": abs(model.body_mass.sum() - 23.259630515567018) < 1e-9,
              "dimensions": (model.nq, model.nv, model.nu) == (13, 12, 0),
              "clearance": abs(clearance - .1) < 2e-6,
              "source_knee_limits": bool(np.all(model.jnt_limited[source.joint_ids[[1, 4]]])),
              "continuous_hips": bool(not np.any(model.jnt_limited[source.joint_ids[[0, 3]]])),
              "no_initial_penetration": all(c["distance_m"] >= -1e-6 for c in initial_contacts),
              "two_wheel_ground_paths": {(c["geom1"], c["geom2"]) for c in initial_contacts}
                  == {("floor", "right_wheel_collision_0"), ("floor", "left_wheel_collision_0")},
              "positive_motor_ratios": min(ratios) > 0,
              "spring_generalized_direction": springs[1] > 0 and springs[4] < 0}
    actuators = source.actuator_module.CoupledActuators(torch, source.hardware, JOINT_ORDER)
    qd = np.zeros(6)
    rows = []
    failure = None
    if all(checks.values()):
        for tick in range(50):
            q = data.qpos[source.qpos_ids].copy()
            error = source.defaults - q
            error[[0, 3]] = np.arctan2(np.sin(error[[0, 3]]), np.cos(error[[0, 3]]))
            request = np.array([40, 60, 0, 40, 60, 0]) * error - np.array([2, 2, .3, 2, 2, .3]) * qd
            with torch.inference_mode():
                active = actuators.apply(torch.tensor(request)[None], torch.tensor(q)[None],
                                         torch.tensor(qd)[None])[0].numpy()
            try:
                passive, spring_lengths = source.spring_torque(q, qd)
            except ValueError as exc:
                failure = str(exc)
                break
            data.qfrc_applied[:] = 0
            data.qfrc_applied[source.qvel_ids] = active + passive
            for _ in range(source.physics_substeps):
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            qd = np.arctan2(np.sin(data.qpos[source.qpos_ids] - q),
                            np.cos(data.qpos[source.qpos_ids] - q)) / source.external_dt
            state = np.r_[data.qpos, data.qvel]
            row = {"time_s": float(data.time), "root_z_m": float(data.qpos[2]),
                   "max_joint_velocity_rad_s": float(np.abs(data.qvel[source.qvel_ids]).max()),
                   "spring_lengths_m": spring_lengths, "active_torque_nm": active.tolist(),
                   "passive_torque_nm": passive.tolist(), "contacts": source.contacts()}
            rows.append(row)
            if not np.isfinite(state).all() or np.abs(data.qvel).max() > 1000:
                failure = "nonfinite_or_unbounded_state"
                break
    checks["short_pd_finite"] = failure is None and len(rows) == 50
    checks = {name: bool(value) for name, value in checks.items()}
    return {"tool": "sample-source-probe", "schema": 1,
            "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "status": "diagnostic_pass" if all(checks.values()) else "failed",
            "checks": checks, "failure": failure, "initial_contacts": initial_contacts,
            "initial_clearance_m": clearance, "mass_kg": float(model.body_mass.sum()),
            "initial_spring_lengths_m": lengths, "initial_spring_torque_nm": springs.tolist(),
            "initial_knee_ratios": ratios, "timing": {"physics_dt_s": model.opt.timestep,
                "external_PD_dt_s": source.external_dt, "policy_dt_s": source.policy_dt},
            "model_xml_sha256": hashlib.sha256(source.xml.encode()).hexdigest(),
            "mujoco": mujoco.__version__, "rows": rows,
            "source_runtime_timing_verified": False, "V2_accepted": False,
            "pending": ["PhysX_contact_response_comparison", "original_base_environment_timing",
                        "per_channel_dynamics", "spring_virtual_work_and_contact_tests"],
            "contact_assumptions": {"effective_sliding_friction": .75,
                "rolling_and_torsional_friction": 0,
                "status": "provisional_not_PhysX_calibrated", "adjacent_pairs_excluded": True},
            "training_executed": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--xml", type=Path, required=True)
    args = parser.parse_args()
    report = probe()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    args.xml.parent.mkdir(parents=True, exist_ok=True)
    args.xml.write_text(SampleSource().xml + "\n")
    print(json.dumps({"status": report["status"], "checks": report["checks"], "failure": report["failure"]}))
    return 0 if report["status"] == "diagnostic_pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())

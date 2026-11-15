#!/usr/bin/env python3
"""V5 target-profile static and short bounded-PD diagnostics; never PPO.

Layers the local target parameters (wheel radius 0.060 m, M3508 + 16T->19T
chain, 450 N / 250 mm / 75 mm spring) on the frozen reconstructed source model.
No archive file or frozen source module is modified.
"""

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
from fifty_mujoco.transfer.sample_target import SampleTargetSource, load_profile, total_wheel_ratio


def probe():
    profile = load_profile()
    source = SampleTargetSource()
    model, data = source.model, source.data
    initial_contacts = source.contacts()
    base = model.body("base_link").id
    clearance = bottom_clearance(data.xpos[base, 2], [0, 0, -1],
                                 source.env["env"]["clearance_box_center"],
                                 source.env["env"]["clearance_box_halfsize"])
    springs, lengths = source.spring_torque(source.defaults, np.zeros(6))
    ratios = [source.actuator_module.knee_ratio(sign * source.defaults[k] + math.pi / 2,
                                                source.hardware)
              for k, sign in ((1, 1), (4, -1))]
    wheel_radii = sorted({float(model.geom(name).size[0])
                          for name in ("right_wheel_collision_0", "left_wheel_collision_0")})
    stop_lengths = {}
    for joint, index in (("rf1_Joint", 1), ("lf1_Joint", 4)):
        q = source.defaults.copy()
        lower, upper = model.jnt_range[source.joint_ids[index]]
        q[index] = lower if joint == "rf1_Joint" else upper
        _, stop_lengths[joint] = source.spring_torque(q, np.zeros(6))
    checks = {"mass": abs(model.body_mass.sum() - 23.259630515567018) < 1e-9,
              "dimensions": (model.nq, model.nv, model.nu) == (13, 12, 0),
              "target_wheel_radius": wheel_radii == [profile["wheel"]["radius_m"]],
              "clearance": abs(clearance - profile["environment"]["expected_initial_clearance_m"]) < 2e-6,
              "source_knee_limits": bool(np.all(model.jnt_limited[source.joint_ids[[1, 4]]])),
              "knee_stop_equals_spring_minimum": all(
                  abs(stop_lengths[name][0 if name == "rf1_Joint" else 1]
                      - profile["spring"]["min_length_m"]) < 1e-6 for name in stop_lengths),
              "continuous_hips": bool(not np.any(model.jnt_limited[source.joint_ids[[0, 3]]])),
              "no_initial_penetration": all(c["distance_m"] >= -1e-6 for c in initial_contacts),
              "two_wheel_ground_paths": {(c["geom1"], c["geom2"]) for c in initial_contacts}
                  == {("floor", "right_wheel_collision_0"), ("floor", "left_wheel_collision_0")},
              "positive_motor_ratios": min(ratios) > 0,
              "spring_generalized_direction": springs[1] > 0 and springs[4] < 0}
    actuators = source.actuator_module.CoupledActuators(torch, source.hardware, JOINT_ORDER)
    rows, failure = [], None
    stroke = [profile["spring"]["min_length_m"] - profile["spring"]["length_limit_tolerance_m"],
              profile["spring"]["max_length_m"] + profile["spring"]["length_limit_tolerance_m"]]
    if all(checks.values()):
        qd = np.zeros(6)
        for _ in range(50):
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
            rows.append({"time_s": float(data.time), "root_z_m": float(data.qpos[2]),
                         "spring_lengths_m": spring_lengths,
                         "active_torque_nm": active.tolist(),
                         "passive_torque_nm": passive.tolist()})
            if not np.isfinite(np.r_[data.qpos, data.qvel]).all():
                failure = "nonfinite_state"
                break
    checks["short_pd_finite"] = failure is None and len(rows) == 50
    checks["spring_stroke_respected"] = bool(rows) and all(
        stroke[0] <= length <= stroke[1] for row in rows for length in row["spring_lengths_m"])
    checks = {name: bool(value) for name, value in checks.items()}
    return {"tool": "sample-target-probe", "schema": 1,
            "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "status": "target_diagnostic_pass" if all(checks.values()) else "failed",
            "target_profile": {"schema": profile["schema"],
                               "wheel_radius_m": profile["wheel"]["radius_m"],
                               "wheel_total_rotor_to_wheel_ratio": total_wheel_ratio(profile),
                               "spring_force_n": profile["spring"]["reference_force_n"],
                               "spring_stroke_m": profile["spring"]["stroke_m"],
                               "spawn_height_delta_m": profile["environment"]["spawn_height_delta_m"]},
            "checks": checks, "failure": failure, "initial_contacts": initial_contacts,
            "initial_clearance_m": clearance, "mass_kg": float(model.body_mass.sum()),
            "initial_spring_lengths_m": lengths, "initial_spring_torque_nm": springs.tolist(),
            "knee_stop_spring_lengths_m": {name: values[0 if name == "rf1_Joint" else 1]
                                           for name, values in stop_lengths.items()},
            "initial_knee_ratios": ratios,
            "timing": {"physics_dt_s": model.opt.timestep,
                       "external_PD_dt_s": source.external_dt, "policy_dt_s": source.policy_dt},
            "model_xml_sha256": hashlib.sha256(source.xml.encode()).hexdigest(),
            "profile_sha256": hashlib.sha256(json.dumps(profile, sort_keys=True).encode()).hexdigest(),
            "mujoco": mujoco.__version__, "rows": rows,
            "target_model_accepted": False, "V5_accepted": False,
            "pending": ["five_class_command_regression", "transition_and_turning_protocol",
                        "wheel_inertia_review", "spring_force_progression",
                        "archived_URDF_effort_velocity_limit_review"],
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
    args.xml.write_text(SampleTargetSource().xml + "\n")
    print(json.dumps({"status": report["status"], "checks": report["checks"],
                      "failure": report["failure"]}))
    return 0 if report["status"] == "target_diagnostic_pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())

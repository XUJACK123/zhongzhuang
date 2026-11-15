#!/usr/bin/env python3
"""Exploratory frozen-policy CPU MuJoCo trials; never formal V3/V4 acceptance."""

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
from fifty_mujoco.contracts.sample_actor31 import (
    JOINT_ORDER, action_targets, advance_reference, bottom_clearance,
    build_observation, clip_actions, ramp_commands,
)
from fifty_mujoco.transfer.sample_checkpoint import load_sample_actor
from fifty_mujoco.transfer.sample_source import SampleSource, verified_artifact


def trial(source, actor, command, seed, seconds):
    source.reset()
    model, data = source.model, source.data
    base = model.body("base_link").id
    rng = np.random.default_rng(seed)
    noise = source.env["env"].get("initial_velocity_noise", 0)
    # Source _reset_root_states perturbs horizontal linear and body angular
    # velocities only. Joint pose, root pose and clearance remain deterministic.
    data.qvel[:2] = rng.uniform(-noise, noise, 2)
    data.qvel[3:6] = rng.uniform(-noise, noise, 3)
    mujoco.mj_forward(model, data)
    reference, heading = data.qpos[:2].tolist(), 0.
    applied = [0., 0., .1]
    last_action = [0.] * 6
    joint_velocity = np.zeros(6)
    body_velocity = np.zeros(3)
    actuators = source.actuator_module.CoupledActuators(torch, source.hardware, JOINT_ORDER)
    limits = model.jnt_range[source.joint_ids[[1, 4]]].tolist()
    saturated = np.zeros(6, dtype=int)
    controls = 0
    trace = []
    errors = []
    failure = None
    for tick in range(round(seconds/source.policy_dt)):
        rotation = data.xmat[base].reshape(3, 3)
        gravity = rotation.T @ np.array([0., 0., -1.])
        velocity6 = np.empty(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, base, velocity6, 1)
        quat = data.qpos[3:7]
        w, x, y, z = quat
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        obs = build_observation(angular_velocity_body=velocity6[:3], projected_gravity_body=gravity,
            applied_commands=applied, joint_position=data.qpos[source.qpos_ids],
            joint_velocity=joint_velocity, previous_clipped_action=last_action,
            position_xy=data.qpos[:2], reference_xy=reference, yaw=yaw,
            reference_yaw=heading, body_velocity_xy=body_velocity[:2])
        with torch.inference_mode():
            action = clip_actions(actor(torch.tensor([obs], dtype=torch.float32))[0].tolist())
        targets = action_targets(action, limits)
        previous_position = data.qpos[:3].copy()
        command_during_step = applied.copy()
        lengths = []
        for _ in range(source.env["control"]["decimation"]):
            q = data.qpos[source.qpos_ids].copy()
            error = np.array(targets["position"]) - q
            error[[0, 3]] = np.arctan2(np.sin(error[[0, 3]]), np.cos(error[[0, 3]]))
            requested = np.array([40, 60, 0, 40, 60, 0])*error + np.array([2, 2, .3, 2, 2, .3])*(
                np.array(targets["velocity"])-joint_velocity)
            with torch.inference_mode():
                torque = actuators.apply(torch.tensor(requested)[None], torch.tensor(q)[None],
                                          torch.tensor(joint_velocity)[None])[0].numpy()
            saturated += (actuators.motor_torque_requested-actuators.motor_torque_applied).abs()[0].numpy() > 1e-6
            controls += 1
            try:
                passive, lengths = source.spring_torque(q, joint_velocity)
            except ValueError as exc:
                failure = "spring_stroke:" + str(exc)
                break
            data.qfrc_applied[:] = 0
            data.qfrc_applied[source.qvel_ids] = torque + passive
            for _ in range(source.physics_substeps):
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            joint_velocity = np.arctan2(np.sin(data.qpos[source.qpos_ids]-q),
                                       np.cos(data.qpos[source.qpos_ids]-q)) / source.external_dt
            if not np.isfinite(np.r_[data.qpos, data.qvel]).all():
                failure = "nonfinite_state"
                break
            knees = data.qpos[source.qpos_ids[[1, 4]]]
            if np.any(knees < np.array(limits)[:, 0]-1e-4) or np.any(knees > np.array(limits)[:, 1]+1e-4):
                failure = "source_knee_stop_violation"
                break
        rotation = data.xmat[base].reshape(3, 3)
        gravity = rotation.T @ np.array([0., 0., -1.])
        body_velocity = rotation.T @ ((data.qpos[:3]-previous_position)/source.policy_dt)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, base, velocity6, 1)
        clearance = bottom_clearance(data.qpos[2], gravity,
            source.env["env"]["clearance_box_center"], source.env["env"]["clearance_box_halfsize"])
        tilt = math.degrees(math.acos(float(np.clip(-gravity[2], -1, 1))))
        if failure is None and (tilt > 60 or data.qpos[2] < .15):
            failure = "tilt_or_low_root"
        if failure is None:
            base_contact_force = np.zeros(3)
            for contact_index, c in enumerate(data.contact):
                names = [model.geom(i).name for i in (c.geom1, c.geom2)]
                if "floor" in names and "base_link_collision_0" in names:
                    force = np.empty(6)
                    mujoco.mj_contactForce(model, data, contact_index, force)
                    # All ground normals share world +Z; tangent basis may
                    # differ, so rotate each contact force before summation.
                    base_contact_force += np.array(c.frame).reshape(3, 3).T @ force[:3]
            if np.linalg.norm(base_contact_force) > 10:
                failure = "base_ground_contact"
        yaw_error = velocity6[2]-command_during_step[1]
        vx_error = body_velocity[0]-command_during_step[0]
        if data.time >= 2:
            errors.append([vx_error, yaw_error, abs(clearance-command_during_step[2]), tilt])
        if lengths:
            trace.append([data.time, *data.qpos, *data.qvel, *command_during_step,
                          *action, *torque, *passive, *lengths, clearance, tilt,
                          *obs, *joint_velocity])
        if failure:
            break
        reference, heading = advance_reference(reference, heading, applied[0], applied[1], source.policy_dt)
        applied = ramp_commands(applied, command, source.policy_dt)
        last_action = action
    values = np.asarray(errors)
    return {"seed": seed, "command": command, "completed": failure is None and data.time >= seconds-1e-8,
            "elapsed_s": float(data.time), "first_failure": failure,
            "metrics_after_2s": {"samples": len(errors),
                "root_vx_rmse": float(np.sqrt(np.mean(values[:, 0]**2))) if len(errors) else None,
                "yaw_rmse": float(np.sqrt(np.mean(values[:, 1]**2))) if len(errors) else None,
                "clearance_mae": float(values[:, 2].mean()) if len(errors) else None,
                "total_tilt_p95_deg": float(np.percentile(values[:, 3], 95)) if len(errors) else None},
            "motor_saturation_fraction": (saturated/max(1, controls)).tolist()}, np.asarray(trace)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", default="model_3050.pt")
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0 or not 1 <= args.seeds <= 40:
        parser.error("positive finite duration and 1..40 seeds required")
    torch.set_num_threads(1)
    actor, metadata = load_sample_actor(verified_artifact(args.checkpoint))
    source = SampleSource()
    rows, traces = [], {}
    for i, (name, command) in enumerate({"stand": [0, 0, .1], "forward": [.15, 0, .1],
            "backward_ood": [-.05, 0, .1], "left": [0, .3, .1], "right": [0, -.3, .1]}.items()):
        for j in range(args.seeds):
            seed = 10000 + 100*i + j
            row, trace = trial(source, actor, command, seed, args.seconds)
            row["scenario"] = name
            rows.append(row)
            traces[f"{name}_{seed}"] = trace
        print(json.dumps({"scenario": name, "completed": sum(r["completed"] for r in rows if r["scenario"] == name),
                          "trials": args.seeds}), flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    trace_path = args.out.with_suffix(".npz")
    np.savez_compressed(trace_path, **traces)
    report = {"tool": "sample-rollout-diagnostic", "schema": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": "diagnostic_only", "checkpoint": args.checkpoint, **metadata,
        "checkpoint_sha256": hashlib.sha256(verified_artifact(args.checkpoint).read_bytes()).hexdigest(),
        "completed": sum(r["completed"] for r in rows), "trials": len(rows), "duration_s": args.seconds,
        "V3_accepted": False, "training_executed": False,
        "randomization_enabled": False, "initial_velocity_noise": source.env["env"].get("initial_velocity_noise", 0),
        "rows": rows, "trace": {"file": trace_path.name, "sha256": hashlib.sha256(trace_path.read_bytes()).hexdigest(),
            "columns": ["time", "qpos[13]", "qvel[12]", "applied_command[3]", "action[6]",
                        "active_generalized_torque[6]", "passive_generalized_torque[6]", "spring_length[2]",
                        "clearance", "tilt", "action_start_observation[31]", "end_joint_velocity_estimate[6]"],
            "state_timestamp": "post-step; action/observation timestamp is step start; torque/length is last external PD update"},
        "limitations": ["Original base environment revision/timing missing; finite differences reconstructed from local candidate.",
                        "MuJoCo contact parameters provisional; no PhysX response parity.",
                        "Root forward velocity metric; command-point offset not applied to velocity error.",
                        "Strict first failure; no automatic reset or duration stitching.",
                        "These diagnostic seeds are excluded from future acceptance sets."]}
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"completed": report["completed"], "trials": len(rows), "V3_accepted": False}))


if __name__ == "__main__":
    main()

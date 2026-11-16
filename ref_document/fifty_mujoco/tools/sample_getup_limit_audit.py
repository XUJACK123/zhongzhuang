#!/usr/bin/env python3
"""Trace inherited spring limits and explore a separate geometric candidate.

The candidate is diagnostic only. Production profiles, firmware and training
limits remain untouched; component collisions and actual stops need evidence.
"""
import json
from pathlib import Path
import sys
import math
import numpy as np
import mujoco

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/"src"))
from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31
from fifty_mujoco.envs.sample_env31_hardware import rpy_from_wxyz
from fifty_mujoco.transfer.sample_source import verified_artifact


def main():
    e = GuardedHardwareEnv31(reset_mode="natural", wheel_kd=.14)
    archived = json.loads(verified_artifact("gas_spring_resolved.json").read_text())
    scans, candidate = [], []
    for i, spring in enumerate(e.source.spring_config["springs"]):
        sign = 1 if i == 0 else -1
        interval, detail = e.source.spring_module.stroke_joint_interval(spring, e.source.defaults[1+3*i], sign)
        candidate.append(interval)
        old = archived["installation_measurements"]["stroke_consistency"]["per_leg_model"][i]
        scans.append({"joint": spring["joint"], "current_limits_rad": e.limits[i].tolist(),
                      "target_spring_only_branch_interval_rad": interval, "target_analysis": detail,
                      "inherited_source_spring_extension_m": old["modeled_extended_hole_distance_m"],
                      "source_spring_interval_rad": old["corrected_joint_limits_rad"],
                      "target_extension_m": spring["max_length_m"]})
    rows = []
    for mode in ("current", "target_spring_geometry_candidate"):
        for pose in ("natural", "default"):
            env = GuardedHardwareEnv31(reset_mode=pose, wheel_kd=.14)
            env.reset(200080)
            env.model.opt.timestep = .00025
            limits = env.limits.copy() if mode == "current" else np.array(candidate)
            env.model.jnt_range[env.source.joint_ids[[1, 4]]] = limits
            trace = []
            failure = None
            for tick in range(12000):
                q, qd = env.data.qpos[env.source.qpos_ids], env.data.qvel[env.source.qvel_ids]
                try:
                    passive, lengths = env.source.spring_torque(q, qd)
                except ValueError as exc:
                    failure = "spring_stroke:"+str(exc)
                    break
                env.data.qfrc_applied[:] = 0.
                env.data.qfrc_applied[env.source.qvel_ids] = passive
                mujoco.mj_step(env.model, env.data)
                mujoco.mj_forward(env.model, env.data)
                knees = env.data.qpos[env.source.qpos_ids[[1, 4]]]
                rpy = np.degrees(rpy_from_wxyz(env.data.qpos[3:7]))
                if np.any(knees < limits[:, 0]-1e-4) or np.any(knees > limits[:, 1]+1e-4):
                    failure = "joint_limit"
                elif max(abs(rpy[:2])) > 40:
                    failure = "upright_scope_exceeded"
                if tick % 8 == 0 or failure:
                    trace.append({"time_s": float(env.data.time), "knees_rad": knees.tolist(),
                                  "lengths_m": lengths, "roll_pitch_deg": rpy[:2].tolist(),
                                  "root_height_m": float(env.data.qpos[2]),
                                  "contacts": [[env.model.geom(int(x)).name for x in (ct.geom1, ct.geom2)] for ct in env.data.contact]})
                if failure:
                    break
            rows.append({"variant": mode, "pose": pose, "time_s": float(env.data.time),
                         "failure": failure, "limits_rad": limits.tolist(), "trace": trace})
    report = {"schema": "target_spring_limit_audit_v1", "evidence_kind": "simulation_and_archived_configuration",
              "limits": scans, "passive_trials": rows,
              "body_masses_kg": {e.model.body(i).name: float(e.model.body_mass[i]) for i in range(1, e.model.nbody)},
              "joint_armature": e.model.dof_armature.tolist(),
              "candidate_promoted": False, "production_limits_changed": False,
              "interpretation": "The inherited extension stop comes from the archived ~228 mm installed spring, while the current 250 mm spring-only geometry allows a wider branch. This is a modeling discrepancy, not permission to widen hardware limits. The separate candidate still requires full component-contact and physical-stop validation."}
    out = PROJECT/"reports/sample_weight/getup_limit_audit_20261116.json"
    if out.exists():
        raise ValueError("output already exists")
    out.write_text(json.dumps(report, indent=2))
    print(json.dumps({"limits": scans, "trials": [{k:r[k] for k in ("variant", "pose", "time_s", "failure")} for r in rows]}, indent=2))


if __name__ == "__main__":
    main()

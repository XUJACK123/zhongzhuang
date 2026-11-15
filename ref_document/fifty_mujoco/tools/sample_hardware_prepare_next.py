#!/usr/bin/env python3
"""Prepare provenance and reset specifications using only the Python standard library.

Run from any directory. Reads saved files, including interrupted ARM RAM dumps.
Outputs are separate derived reports. Timestamps from watch.tcl are requested
debugger intervals, while MCU ticks reflect firmware time. RAM symbols were read
sequentially, so these records cannot identify millisecond actuator dynamics.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import struct
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from fifty_mujoco.contracts.sample_actor31 import ACTOR_SIZES, SOURCE_DEFAULT_ANGLES

PLAN = "configs/hardware31_next_training_20261115.json"
SESSION = "diagnostics/commission_20261113_230207"
LABELS = ("right_hip_DM4", "right_knee_DM3", "right_wheel_M3508_3",
          "left_hip_DM1", "left_knee_DM2", "left_wheel_M3508_2")
SIZES = {"g_chassis": 40, "g_imu": 128, "g_measurement": 92,
         "g_motor_snapshot": 248, "g_rl": 1164}


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def decode_sample(blobs):
    """Decode the saved H723 ABI, rejecting incomplete or non-finite records."""
    for name, size in SIZES.items():
        if len(blobs[name]) != size:
            raise ValueError(f"{name}: expected {size} bytes, got {len(blobs[name])}")
    chassis = struct.unpack("<10I", blobs["g_chassis"])
    bank = blobs["g_motor_snapshot"]
    result = {
        "mcu_tick_ms": chassis[0], "inhibit": chassis[1], "fault": chassis[2],
        "mode": chassis[8],
        "q_rad": list(struct.unpack_from("<6f", blobs["g_measurement"])),
        "requested_motor_torque_nm": list(struct.unpack_from("<6f", blobs["g_rl"], 1140)),
        "yaw_rad": struct.unpack_from("<f", blobs["g_imu"], 52)[0],
        "encoder_total_counts": [struct.unpack_from("<q", bank, i * 40 + 8)[0] for i in range(6)],
        "feedback_raw_firmware_units": [struct.unpack_from("<f", bank, i * 40 + 24)[0] for i in range(6)],
        "feedback_received": [struct.unpack_from("<I", bank, i * 40 + 32)[0] for i in range(6)],
    }
    floats = result["q_rad"] + result["requested_motor_torque_nm"] + result["feedback_raw_firmware_units"] + [result["yaw_rad"]]
    if not all(math.isfinite(v) for v in floats):
        raise ValueError("non-finite RAM value")
    result["possible_count_tearing"] = any(abs(v) > 2**31 for v in result["encoder_total_counts"])
    return result


def recover_watch(directory, workspace):
    script = (directory / "watch.tcl").read_text(encoding="utf-8-sig")
    count = int(re.search(r"set n (\d+)", script).group(1))
    interval_ms = int(re.search(r"sleep (\d+)", script).group(1))
    samples, rejected, inputs = [], [], []
    indices = sorted({int(p.name.split("_")[0]) for p in directory.glob("*_g_*.bin")})
    for index in indices:
        blobs = {}
        for name in SIZES:
            path = directory / f"{index:03d}_{name}.bin"
            if path.exists():
                blobs[name] = path.read_bytes()
                inputs.append({"path": path.relative_to(workspace).as_posix(), "sha256": sha256(path)})
        try:
            sample = decode_sample(blobs)
        except (KeyError, ValueError) as exc:
            rejected.append({"index": index, "reason": str(exc)})
            continue
        sample.update(index=index, requested_debugger_time_s=index * interval_ms / 1000)
        samples.append(sample)
    return {
        "schema": "saved_arm_watch_recovery_v1",
        "source_directory": directory.relative_to(workspace).as_posix(),
        "requested_samples": count, "decoded_samples": len(samples),
        "missing_indices": sorted(set(range(count)) - set(indices)),
        "rejected": rejected, "samples": samples, "raw_inputs": inputs,
        "watch_script_sha256": sha256(directory / "watch.tcl"),
        "mcu_tick_span_s": (samples[-1]["mcu_tick_ms"] - samples[0]["mcu_tick_ms"]) / 1000 if samples else None,
        "openocd_log": (directory / "openocd.log").relative_to(workspace).as_posix(),
        "openocd_log_present": (directory / "openocd.log").is_file(),
        "limitations": ["sequential_debugger_reads_not_atomic", "requested_interval_not_actual_sample_time",
                        "feedback_units_from_motor_io_c_DM_reported_mNm_scaled_to_Nm_DJI_current_A",
                        "DM_self_report_is_not_independent_shaft_torque_measurement",
                        "interrupted_run_not_a_complete_identification_trial"],
    }


def validate_plan(plan, profile):
    baseline = plan["baseline"]
    if tuple(baseline["actor_sizes"]) != ACTOR_SIZES:
        raise ValueError("actor architecture differs from the 31D contract")
    expected_timing = {"physics": "physics_dt_s", "pd": "external_pd_dt_s",
                       "estimator": "estimator_dt_s", "actor": "policy_dt_s"}
    for key, field in expected_timing.items():
        if baseline["timing_s"][key] != profile[field]:
            raise ValueError(f"timing mismatch: {key}")
    ranges = list(plan["evaluation"]["seed_ranges_inclusive"].values())
    for i, (low, high) in enumerate(ranges):
        if not isinstance(low, int) or not isinstance(high, int) or low > high:
            raise ValueError("invalid inclusive seed range")
        if any(max(low, a) <= min(high, b) for a, b in ranges[:i]):
            raise ValueError("seed ranges overlap")
    pose = profile["natural_joint_pose_rad"]
    for index, limits in zip((1, 4), profile["firmware_target_knee_limits_rad"]):
        if not limits[0] <= pose[index] <= limits[1]:
            raise ValueError("surveyed knee pose exceeds configured limits")


def can_pair_summary(data):
    a, b = data["a"], data["b"]
    before = {n["label"]: n for n in a["nodes"]}
    after = {n["label"]: n for n in b["nodes"]}
    deltas = {name: after[name]["received"] - before[name]["received"] for name in LABELS}
    tick_delta = b["chassis"]["tick"] - a["chassis"]["tick"]
    return {"tick_delta_ms": tick_delta, "received_deltas": deltas,
            "all_six_increasing": tick_delta > 0 and all(v > 0 for v in deltas.values()),
            "inhibit_last": b["chassis"]["inhibit"]}


def prepare(project, out):
    workspace = project.parents[1]
    chassis = project.parent / "chassis_ref_new"
    plan = read_json(project / PLAN)
    profile_path = project / plan["baseline"]["profile"]
    profile = read_json(profile_path)
    validate_plan(plan, profile)
    checkpoint = project / plan["baseline"]["checkpoint"]
    if sha256(checkpoint) != plan["baseline"]["sha256"]:
        raise ValueError("baseline checkpoint hash mismatch")
    out.mkdir(parents=True, exist_ok=True)
    recoveries = {}
    for name in ("watch_left_leg_under_torque", "watch_freeair_cap20"):
        report = recover_watch(chassis / SESSION / name, workspace)
        target = out / f"hardware_{name}_recovered_20261115.json"
        target.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        recoveries[name] = {"report": target.relative_to(workspace).as_posix(),
                            "decoded_samples": report["decoded_samples"],
                            "rejected_samples": len(report["rejected"]),
                            "openocd_log_present": report["openocd_log_present"]}
    reset_bank = {
        "schema": "hardware_getup_reset_spec_v1", "simulation_validated": False,
        "joint_order": list(LABELS), "standing_reference_q_rad": list(SOURCE_DEFAULT_ANGLES),
        "entries": [{"name": "surveyed_supported_20261113", "q_rad": profile["natural_joint_pose_rad"],
                     "hip_axis_height_m": 0.300, "hip_to_wheel_distance_m": 0.350,
                     "wheel_center_behind_hip": True,
                     "pose_source": "tape_measurements_plus_kinematic_inverse_not_six_direct_angle_measurements",
                     "external_support_present_at_measurement": True,
                     "root_height_requires_collision_geometry_conversion": True,
                     "eligible_as_unsupported_rest_reset": False,
                     "evidence": "ref_document/chassis_ref_new/docs/COMMISSIONING_LOG_20261113.md#2355-写入标定并放行-motor_calibrated"}],
        "pending": ["generate_zero_torque_settled_states_with_explicit_contacts",
                    "validate_spring_stroke_linkage_branch_and_nonpenetration",
                    "collect_repeatable_unsupported_physical_rest_states"],
    }
    reset_file = out / "hardware_getup_reset_bank_20261115.json"
    reset_file.write_text(json.dumps(reset_bank, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    evidence = [project / PLAN, profile_path, checkpoint,
                chassis / "user/robot_config.h", chassis / "user/state_estimator.c",
                chassis / "user/motor_io.h", chassis / "user/motor_io.c",
                chassis / "user/algorithm/rl.c", chassis / "docs/COMMISSIONING_LOG_20261113.md",
                project / "reports/sample_weight/hardware_final_acceptance_20261115.json",
                project / "reports/sample_weight/hardware_final_startup_20261115.json"]
    can_records = {}
    for stamp in ("20261114_000202", "20261114_010445"):
        directory = chassis / "diagnostics" / f"live_can_{stamp}"
        evidence.extend([directory / "result.json", directory / "openocd.log"])
        can_records[stamp] = can_pair_summary(read_json(directory / "result.json"))
    watches = {}
    for name in ("watch_verify_signs", "watch_yaw360", "watch_odometry1m", "watch_odometry1m_b", "watch_odometry1m_c"):
        directory = chassis / SESSION / name
        path = directory / "watch.json"
        evidence.extend([path, directory / "openocd.log"])
        rows = read_json(path)["samples"]
        yaw = sum(math.atan2(math.sin(b["yaw"] - a["yaw"]), math.cos(b["yaw"] - a["yaw"]))
                  for a, b in zip(rows, rows[1:]))
        delta_xy = None
        if "xy" in rows[0] and "xy" in rows[-1]:
            delta_xy = [b - a for a, b in zip(rows[0]["xy"], rows[-1]["xy"])]
        watches[name] = {"samples": len(rows), "yaw_unwrapped_deg": math.degrees(yaw),
                         "estimator_delta_xy_m": delta_xy,
                         "not_ground_truth_displacement": True}
    recovered = read_json(out / "hardware_watch_left_leg_under_torque_recovered_20261115.json")
    ratios = [abs(row["feedback_raw_firmware_units"][4] / row["requested_motor_torque_nm"][4])
              for row in recovered["samples"] if row["mode"] == 3 and row["inhibit"] == 0
              and row["feedback_received"][4] > 0 and abs(row["requested_motor_torque_nm"][4]) > 1]
    audit = {
        "schema": "hardware_next_offline_preparation_v1", "date": "2026-11-15",
        "status": "offline_preparation_passed_physics_validation_pending",
        "inputs": [{"path": p.relative_to(workspace).as_posix(), "sha256": sha256(p)} for p in evidence],
        "can_historical_pairs": can_records, "watch_summaries": watches,
        "recovered_partial_watches": recoveries,
        "left_knee_reported_feedback_to_request_median": statistics.median(ratios) if ratios else None,
        "left_knee_ratio_eligible_active_samples": len(ratios),
        "independent_torque_calibrated": False,
        "checks": {"checkpoint_hash": True, "profile_timing": True, "disjoint_planned_seed_ranges": True,
                   "surveyed_knees_inside_limits": True, "actor_architecture": True},
        "reset_bank": reset_file.relative_to(workspace).as_posix(),
        "pending_runtime": ["CPU_getup_adapter_contact_and_reset_dynamics_validation",
                            "CPU_Torch_actor_initialization_and_PPO_smoke",
                            "getup_training_and_heldout_evaluation"],
        "hardware_accessed": False, "simulator_run": False, "gpu_used": False,
    }
    target = out / "hardware_next_offline_20261115.json"
    target.write_text(json.dumps(audit, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=PROJECT)
    args = parser.parse_args()
    project = args.project.resolve()
    report = prepare(project, project / "reports/sample_weight")
    print(json.dumps({"status": report["status"], "checks": report["checks"],
                      "recovery": report["recovered_partial_watches"]}, indent=2))


if __name__ == "__main__":
    main()

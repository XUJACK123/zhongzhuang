"""Strict JSONL protocol records for get-up replay and later synchronized capture."""

from dataclasses import asdict
import math

from ..contracts.getup31 import GetupController, Sample, SCHEMA, blended_targets
from ..contracts.sample_actor31 import finite_vector
from ..learning.getup_curriculum import plan_digest

KINDS = ("synthetic_protocol", "simulation", "hardware_synced", "hardware_debugger")
HARD_FAILURES = ("joint_limit", "spring_stroke", "linkage_branch", "forbidden_contact")


def validate_header(header, plan):
    if header["schema"] != SCHEMA or header["record_type"] != "header":
        raise ValueError("get-up trace header/schema mismatch")
    if header["evidence_kind"] not in KINDS:
        raise ValueError("unknown trace evidence kind")
    if header["plan_sha256"] != plan_digest(plan):
        raise ValueError("trace uses a different protocol plan")
    if type(header["seed"]) is not int or not isinstance(header["reset_class"], str):
        raise ValueError("trace seed and reset class required")
    if header["balance_checkpoint_sha256"] != plan["baseline"]["sha256"]:
        raise ValueError("balance checkpoint differs from frozen baseline")
    digest = header["getup_checkpoint_sha256"]
    if header["evidence_kind"] != "synthetic_protocol" and (
            not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest)):
        raise ValueError("get-up checkpoint SHA-256 required")


def validate_raw_telemetry(raw):
    """Common synchronized capture fields, with explicit source-side clocks."""
    for name in ("mcu_tick_ms", "mode", "inhibit"):
        if type(raw[name]) is not int or raw[name] < 0:
            raise ValueError(f"{name} must be a nonnegative integer")
    counters = raw["can_received"]
    if len(counters) != 6 or any(type(v) is not int or v < 0 for v in counters):
        raise ValueError("six CAN counters required")
    ticks = raw["can_last_tick_ms"]
    if len(ticks) != 6 or any(type(value) is not int or not 0 <= value < 2**32 for value in ticks):
        raise ValueError("six CAN feedback timestamps required")
    now = raw["mcu_tick_ms"]
    ages = [(now - value) % 2**32 for value in ticks]
    if any(age > 30 for age in ages):
        raise ValueError("CAN feedback exceeds firmware freshness threshold")
    for key, limit in (("imu_tick_ms", 5), ("remote_tick_ms", 100)):
        value = raw[key]
        if type(value) is not int or not 0 <= value < 2**32 or (now - value) % 2**32 > limit:
            raise ValueError(f"stale or invalid {key}")
    for name, size in (("joint_velocity_rad_s", 6), ("requested_motor_torque_nm", 6),
                       ("dm_reported_torque_nm", 4), ("wheel_reported_current_a", 2),
                       ("imu_quaternion_wxyz", 4), ("projected_gravity", 3), ("estimated_xy_m", 2)):
        finite_vector(raw[name], size, name)
    norm = sum(v * v for v in raw["imu_quaternion_wxyz"])
    if not math.isclose(norm, 1., abs_tol=.02):
        raise ValueError("quaternion norm outside capture tolerance")
    host_time = raw["host_monotonic_s"]
    voltage = raw["battery_voltage_v"]
    if isinstance(host_time, bool) or not math.isfinite(host_time) or host_time < 0:
        raise ValueError("invalid host_monotonic_s")
    if voltage is not None and (isinstance(voltage, bool) or not math.isfinite(voltage) or voltage <= 0):
        raise ValueError("battery voltage must be positive or null when unmeasured")


def replay(header, records, plan, knee_limits):
    validate_header(header, plan)
    controller = GetupController(plan)
    previous_raw_tick = None
    first_time = None
    decisions = []
    outcome = None
    for index, record in enumerate(records):
        if outcome is not None:
            raise ValueError("one trial per trace: records present after terminal outcome")
        if record["record_type"] != "sample":
            raise ValueError("sample record required")
        sample = Sample(**record["signals"])
        if first_time is None:
            first_time = sample.time_s
        if header["evidence_kind"] == "hardware_synced":
            raw = record["raw_telemetry"]
            validate_raw_telemetry(raw)
            if previous_raw_tick is not None and raw["mcu_tick_ms"] <= previous_raw_tick:
                raise ValueError("MCU ticks do not increase within this trial")
            previous_raw_tick = raw["mcu_tick_ms"]
        decision = controller.update(sample, record["enabled"])
        target = blended_targets(decision, record["getup_action"], record["balance_action"], knee_limits)
        decisions.append({"index": index, "time_s": sample.time_s, **asdict(decision), "targets": target})
        if decision.failure is not None or decision.success:
            outcome = decision
    if not decisions:
        raise ValueError("empty trace")
    failure = outcome.failure if outcome is not None else "incomplete_trace"
    return {"schema": SCHEMA, "evidence_kind": header["evidence_kind"], "seed": header["seed"],
            "reset_class": header["reset_class"], "plan_sha256": header["plan_sha256"],
            "getup_checkpoint_sha256": header["getup_checkpoint_sha256"],
            "balance_checkpoint_sha256": header["balance_checkpoint_sha256"],
            "success": outcome is not None and outcome.success,
            "failure": failure, "hard_limit_failure": failure in HARD_FAILURES,
            "samples": len(decisions), "elapsed_s": decisions[-1]["time_s"] - first_time,
            "decisions": decisions,
            "physical_validation_eligible": header["evidence_kind"] == "hardware_synced",
            "simulation_validation_eligible": header["evidence_kind"] == "simulation"}

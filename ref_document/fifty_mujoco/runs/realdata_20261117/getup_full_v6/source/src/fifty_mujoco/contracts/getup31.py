"""Simulator-independent get-up lifecycle and action contract.

All timestamps are monotonic seconds from the same clock. The plant supplies
contact and mechanical validity signals. STOP and FAILED require zero active
motor torque. Passive spring forces remain part of the plant.
"""

from dataclasses import dataclass
from enum import Enum
import math

from .sample_actor31 import LEG_INDICES, SOURCE_DEFAULT_ANGLES, action_targets, clip_actions, finite_vector, wrap_angle

SCHEMA = "hardware_getup31_protocol_v1"


class Phase(str, Enum):
    STOP = "STOP"
    GETUP = "GETUP"
    HANDOFF = "HANDOFF"
    BALANCE = "BALANCE"
    FAILED = "FAILED"


@dataclass(frozen=True)
class Sample:
    time_s: float
    joint_position_rad: tuple
    roll_pitch_deg: tuple
    angular_velocity_rad_s: tuple
    forward_velocity_m_s: float
    clearance_m: float
    displacement_m: float
    both_wheels_supported: bool
    body_contact: bool
    assistance_force_n: float
    telemetry_valid: bool
    joint_limits_valid: bool
    spring_stroke_valid: bool
    linkage_branch_valid: bool
    forbidden_contact: bool
    balance_failure: str | None = None

    def __post_init__(self):
        for name, size in (("joint_position_rad", 6), ("roll_pitch_deg", 2), ("angular_velocity_rad_s", 3)):
            object.__setattr__(self, name, tuple(finite_vector(getattr(self, name), size, name)))
        for name in ("time_s", "forward_velocity_m_s", "clearance_m", "displacement_m", "assistance_force_n"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
        if min(self.time_s, self.displacement_m, self.assistance_force_n) < 0:
            raise ValueError("time, displacement and assistance magnitude must be nonnegative")
        for name in ("both_wheels_supported", "body_contact", "telemetry_valid", "joint_limits_valid",
                     "spring_stroke_valid", "linkage_branch_valid", "forbidden_contact"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be boolean")
        if self.balance_failure is not None and (not isinstance(self.balance_failure, str) or not self.balance_failure):
            raise ValueError("balance failure must be a nonempty string or null")


@dataclass(frozen=True)
class Decision:
    phase: Phase
    torque_scale: float
    balance_weight: float
    reset_reference_and_velocity_history: bool
    success: bool
    failure: str | None


class GetupController:
    def __init__(self, plan):
        self.spec = plan["getup"]
        self.criteria = self.spec["success"]
        self.ramp_s = self.spec["enable_ramp_s"][0]
        self.gap_limit_s = 2.5 * plan["baseline"]["timing_s"]["actor"]
        durations = [self.ramp_s, self.spec["getup_timeout_s"], self.spec["stable_dwell_s"],
                     self.spec["handoff_blend_s"], self.spec["post_handoff_balance_s"]]
        if not all(math.isfinite(x) and x > 0 for x in durations):
            raise ValueError("protocol durations must be positive and finite")
        numeric_criteria = [value for value in self.criteria.values() if type(value) is not bool]
        if not all(isinstance(value, (int, float)) and math.isfinite(value) and value > 0 for value in numeric_criteria):
            raise ValueError("success thresholds must be positive and finite")
        if self.spec["stable_dwell_s"] > self.spec["getup_timeout_s"]:
            raise ValueError("stable dwell exceeds get-up timeout")
        self.phase = Phase.STOP
        self.armed = False
        self.previous_time = None
        self.started_at = None
        self.stable_since = None
        self.handoff_at = None
        self.balance_at = None
        self.failure = None
        self.success = False

    def ready(self, sample):
        c = self.criteria
        errors = [sample.joint_position_rad[i] - SOURCE_DEFAULT_ANGLES[i] for i in LEG_INDICES]
        leg_error = max(abs(wrap_angle(error) if index in (0, 3) else error) for index, error in zip(LEG_INDICES, errors))
        return (max(abs(x) for x in sample.roll_pitch_deg) <= c["abs_roll_pitch_deg_max"]
                and max(abs(x) for x in sample.angular_velocity_rad_s) <= c["abs_body_angular_velocity_rad_s_max"]
                and abs(sample.forward_velocity_m_s) <= c["abs_forward_velocity_m_s_max"]
                and abs(sample.clearance_m - self.spec["actor_commands"][2]) <= c["clearance_error_m_max"]
                and leg_error <= c["abs_leg_error_to_balance_default_rad_max"]
                and sample.displacement_m <= c["horizontal_displacement_m_max"]
                and sample.both_wheels_supported and not sample.body_contact
                and sample.assistance_force_n <= 1e-6)

    def _failure(self, sample):
        for valid, reason in ((sample.telemetry_valid, "invalid_telemetry"),
                              (sample.joint_limits_valid, "joint_limit"),
                              (sample.spring_stroke_valid, "spring_stroke"),
                              (sample.linkage_branch_valid, "linkage_branch"),
                              (not sample.forbidden_contact, "forbidden_contact")):
            if not valid:
                return reason
        return None

    def update(self, sample, enabled):
        if type(enabled) is not bool:
            raise ValueError("enabled must be boolean")
        if self.previous_time is not None and sample.time_s <= self.previous_time:
            raise ValueError("timestamps must increase strictly")
        gap = 0 if self.previous_time is None else sample.time_s - self.previous_time
        self.previous_time = sample.time_s
        if not enabled:
            self.phase = Phase.STOP
            self.armed = True
            self.failure = None
            self.success = False
            self.stable_since = None
            return Decision(self.phase, 0., 0., False, False, None)
        if self.phase == Phase.FAILED:
            return Decision(self.phase, 0., 0., False, False, self.failure)
        if self.phase == Phase.STOP and not self.armed:
            return Decision(self.phase, 0., 0., False, False, None)
        failure = self._failure(sample)
        if self.phase != Phase.STOP and gap > self.gap_limit_s + 1e-9:
            failure = failure or "telemetry_time_gap"
        reset_reference = False
        if self.phase == Phase.STOP and failure is None:
            self.phase = Phase.GETUP
            self.started_at = sample.time_s
            self.armed = False
            reset_reference = True
        elapsed = sample.time_s - (self.started_at if self.started_at is not None else sample.time_s)
        if self.phase == Phase.GETUP:
            if elapsed > self.spec["getup_timeout_s"] + 1e-9:
                failure = failure or "getup_timeout"
            if self.ready(sample):
                if self.stable_since is None:
                    self.stable_since = sample.time_s
                if sample.time_s - self.stable_since >= self.spec["stable_dwell_s"] - 1e-9:
                    self.phase = Phase.HANDOFF
                    self.handoff_at = sample.time_s
            else:
                self.stable_since = None
        if self.phase == Phase.HANDOFF:
            if not self.ready(sample):
                failure = failure or "handoff_unstable"
            failure = failure or sample.balance_failure
            if sample.time_s - self.handoff_at >= self.spec["handoff_blend_s"] - 1e-9:
                self.phase = Phase.BALANCE
                self.balance_at = sample.time_s
        if self.phase == Phase.BALANCE:
            failure = failure or sample.balance_failure
            if sample.body_contact:
                failure = failure or "balance_body_contact"
            if not sample.both_wheels_supported or sample.assistance_force_n > 1e-6:
                failure = failure or "balance_support_invalid"
            if sample.displacement_m > self.criteria["horizontal_displacement_m_max"]:
                failure = failure or "balance_displacement"
            self.success = sample.time_s - self.balance_at >= self.spec["post_handoff_balance_s"] - 1e-9
        if failure is not None:
            self.phase = Phase.FAILED
            self.armed = False
            self.failure = failure
            self.success = False
            return Decision(self.phase, 0., 0., False, False, failure)
        weight = 0.
        if self.phase == Phase.HANDOFF:
            fraction = min(1., (sample.time_s - self.handoff_at) / self.spec["handoff_blend_s"])
            weight = fraction * fraction * (3 - 2 * fraction)
        if self.phase == Phase.BALANCE:
            weight = 1.
        return Decision(self.phase, min(1., elapsed / self.ramp_s), weight,
                        reset_reference, self.success, None)


def blended_targets(decision, getup_action, balance_action, knee_limits):
    """Clip actors, blend once, then reuse the existing guarded target mapping.

    Feed clipped_action into the next 31D observation, including during HANDOFF.
    The actuator adapter multiplies active torque by torque_scale AFTER the
    ordinary coupled mapping/envelope. A zero scale requires zero motor command.
    """
    first, second = clip_actions(getup_action), clip_actions(balance_action)
    weight = decision.balance_weight
    if not math.isfinite(weight) or not 0 <= weight <= 1:
        raise ValueError("balance weight outside 0..1")
    action = [(1 - weight) * a + weight * b for a, b in zip(first, second)]
    result = action_targets(action, knee_limits)
    result.update(torque_scale=decision.torque_scale, phase=decision.phase.value)
    return result


def reward_terms(sample, previous_sample, decision, action, previous_action, motor_torque_nm,
                 motor_velocity_rad_s, saturation_fraction, contact_impulse_ns, slip_speed_m_s,
                 weights, success_event=False):
    """Bounded shaping plus logged physical costs, evaluated at policy cadence."""
    action, previous_action = clip_actions(action), clip_actions(previous_action)
    torque = finite_vector(motor_torque_nm, 6, "motor torque")
    speed = finite_vector(motor_velocity_rad_s, 6, "motor speed")
    dt = sample.time_s - previous_sample.time_s
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("positive reward interval required")
    if not math.isfinite(saturation_fraction) or not 0 <= saturation_fraction <= 1:
        raise ValueError("saturation outside 0..1")
    if not all(math.isfinite(x) and x >= 0 for x in (contact_impulse_ns, slip_speed_m_s)):
        raise ValueError("nonnegative contact impulse and slip magnitude required")
    def potential(s):
        upright = math.exp(-sum((x / 20.)**2 for x in s.roll_pitch_deg))
        height = math.exp(-((s.clearance_m - .1) / .05)**2)
        return .5 * (upright + height)
    terms = {
        "progress": .99 * potential(sample) - potential(previous_sample),
        "standing": dt * potential(sample) * float(not sample.body_contact and sample.assistance_force_n <= 1e-6),
        "power": -dt * sum(abs(t * v) for t, v in zip(torque, speed)),
        "saturation": -dt * saturation_fraction,
        "action_rate": -sum((a - b)**2 for a, b in zip(action, previous_action)) / dt,
        "slip": -dt * slip_speed_m_s**2,
        "displacement": -dt * sample.displacement_m**2,
        "contact_impulse": -contact_impulse_ns,
        "success": float(success_event), "failure": -float(decision.failure is not None),
    }
    if set(weights) != set(terms) or not all(math.isfinite(v) and v >= 0 for v in weights.values()):
        raise ValueError("reward weights must match terms and be finite nonnegative")
    weighted = {name: value * weights[name] for name, value in terms.items()}
    return {"raw": terms, "weighted": weighted, "total": sum(weighted.values())}

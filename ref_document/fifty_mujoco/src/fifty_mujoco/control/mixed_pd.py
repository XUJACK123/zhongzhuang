"""Mixed position/velocity PD with motor-only torque limiting (doc section 5.4).

    q*   = q_default + leg_action_scale * a_leg
    qd*  = wheel_action_scale * a_wheel
    tau  = clip(s_tau * [Kp (q* - q) + Kd (qd* - qd)], -tau_max, +tau_max)

The gas spring is a *physical* element: it is applied by the tendon actuator and
is never disabled by ``motor_enabled``.  Optional feed-forward compensation
subtracts the *estimated* spring torque before the motor limit is applied.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np

from ..contracts import names


@dataclass(frozen=True)
class PdGains:
    leg_kp: float = 20.0
    leg_kd: float = 1.0
    wheel_kp: float = 0.0
    wheel_kd: float = 0.2
    torque_limit_nm: float = 30.0
    wheel_torque_limit_nm: float = 5.0
    torque_scale: float = 1.0
    leg_position_scale: float = 0.5
    wheel_velocity_scale: float = 10.0

    def limits(self) -> np.ndarray:
        limit = np.zeros(6)
        limit[list(names.POLICY_LEG_INDICES)] = self.torque_limit_nm
        limit[list(names.POLICY_WHEEL_INDICES)] = self.wheel_torque_limit_nm
        return limit

    @staticmethod
    def target_baseline() -> "PdGains":
        return PdGains()

    @staticmethod
    def teacher_reproduction() -> "PdGains":
        return PdGains(leg_kp=15.0, leg_kd=1.0, wheel_kp=0.0, wheel_kd=0.1, wheel_torque_limit_nm=5.0)


@dataclass(frozen=True)
class ActionTargets:
    position_target: np.ndarray  # (6,) desired joint positions
    velocity_target: np.ndarray  # (6,) desired joint velocities


def decode_action(action: np.ndarray, default_angles: Mapping[str, float], gains: PdGains) -> ActionTargets:
    """Map the six policy channels to joint targets.

    Channel order is the model/policy order (rf0, rf1, r_wheel, lf0, lf1, l_wheel).
    """
    a = np.asarray(action, dtype=float).reshape(6)
    default = np.array([default_angles[name] for name in names.MODEL_JOINT_ORDER], dtype=float)

    position_target = default.copy()
    velocity_target = np.zeros(6)
    for index in names.POLICY_LEG_INDICES:
        position_target[index] = default[index] + gains.leg_position_scale * a[index]
    for index in names.POLICY_WHEEL_INDICES:
        velocity_target[index] = gains.wheel_velocity_scale * a[index]
    return ActionTargets(position_target=position_target, velocity_target=velocity_target)


def compute_torques(
    targets: ActionTargets,
    joint_position: np.ndarray,
    joint_velocity: np.ndarray,
    gains: PdGains,
    estimated_spring_torque: np.ndarray | None = None,
    motor_enabled: bool = True,
) -> np.ndarray:
    """Return the six motor torques actually written to the actuators."""
    position_error = targets.position_target - np.asarray(joint_position, dtype=float)
    velocity_error = targets.velocity_target - np.asarray(joint_velocity, dtype=float)

    kp = np.zeros(6)
    kd = np.zeros(6)
    kp[list(names.POLICY_LEG_INDICES)] = gains.leg_kp
    kd[list(names.POLICY_LEG_INDICES)] = gains.leg_kd
    kp[list(names.POLICY_WHEEL_INDICES)] = gains.wheel_kp
    kd[list(names.POLICY_WHEEL_INDICES)] = gains.wheel_kd

    torque = gains.torque_scale * (kp * position_error + kd * velocity_error)
    if estimated_spring_torque is not None:
        # Compensation happens BEFORE the motor limit, per doc 12.6.
        torque = torque - np.asarray(estimated_spring_torque, dtype=float)
    limits = gains.limits()
    torque = np.clip(torque, -limits, limits)
    if not motor_enabled:
        torque = np.zeros(6)
    return torque

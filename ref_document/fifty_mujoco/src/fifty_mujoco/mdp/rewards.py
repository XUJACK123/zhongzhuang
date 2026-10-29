"""Reward baseline (doc section 6.1).

Every term is ``clip(w_i * f_i, -1, 1) * dt`` and dt is applied exactly once;
the framework must not scale again.
"""

from __future__ import annotations

from typing import Mapping

import numpy as np

#: Migration baseline weights (doc 6.1 table).
DEFAULT_WEIGHTS: dict[str, float] = {
    "tracking_lin_vel": 1.0,
    "tracking_lin_vel_enhance": 1.0,
    "tracking_ang_vel": 1.0,
    "tracking_ang_vel_enhance": 1.0,
    "base_height": 1.0,
    "nominal_state": -1.0,
    "lin_vel_z": -1.0,
    "ang_vel_xy": -0.2,
    "orientation": -100.0,
    "dof_vel": -5e-5,
    "dof_acc": -2.5e-7,
    "torques": -1e-4,
    "action_rate": -0.01,
    "action_smooth": -0.01,
    "collision": -1.0,
    "dof_pos_limits": -1.0,
}


def tracking_lin_vel(command_vx: float, velocity_x: float) -> float:
    return float(np.exp(-((command_vx - velocity_x) ** 2) / 0.25))


def tracking_lin_vel_enhance(command_vx: float, velocity_x: float) -> float:
    return float(np.exp(-((command_vx - velocity_x) ** 2) / 2.5) - 1.0)


def tracking_ang_vel(command_wz: float, velocity_wz: float) -> float:
    return float(np.exp(-((command_wz - velocity_wz) ** 2) / 0.25))


def tracking_ang_vel_enhance(command_wz: float, velocity_wz: float) -> float:
    return float(np.exp(-((command_wz - velocity_wz) ** 2) / 2.5) - 1.0)


def base_height(height: float, command_height: float) -> float:
    return float(np.exp(-((height - command_height) ** 2) / 0.001))


def nominal_state(theta0_left: float, theta0_right: float) -> float:
    return float((theta0_left - theta0_right) ** 2)


def lin_vel_z(velocity_z: float) -> float:
    return float(velocity_z**2)


def ang_vel_xy(ang_vel_x: float, ang_vel_y: float) -> float:
    return float(ang_vel_x**2 + ang_vel_y**2)


def orientation(gravity_x: float, gravity_y: float) -> float:
    return float(gravity_x**2 + gravity_y**2)


def dof_vel(leg_velocities: np.ndarray) -> float:
    return float(np.sum(np.asarray(leg_velocities, dtype=float) ** 2))


def dof_acc(accelerations: np.ndarray) -> float:
    return float(np.sum(np.asarray(accelerations, dtype=float) ** 2))


def torques(motor_torques: np.ndarray) -> float:
    """Motor torques only; the tendon contribution is never counted here."""
    return float(np.sum(np.asarray(motor_torques, dtype=float) ** 2))


def action_rate(action: np.ndarray, previous_action: np.ndarray) -> float:
    return float(np.sum((np.asarray(action, dtype=float) - np.asarray(previous_action, dtype=float)) ** 2))


def action_smooth(action: np.ndarray, previous_action: np.ndarray, previous_previous_action: np.ndarray) -> float:
    a = np.asarray(action, dtype=float)
    b = np.asarray(previous_action, dtype=float)
    c = np.asarray(previous_previous_action, dtype=float)
    return float(np.sum((a - 2.0 * b + c) ** 2))


def collision(illegal_contact_count: int) -> float:
    return float(illegal_contact_count)


def dof_pos_limits(joint_position: np.ndarray, lower: np.ndarray, upper: np.ndarray, margin: float = 0.01) -> float:
    position = np.asarray(joint_position, dtype=float)
    low = np.asarray(lower, dtype=float)
    high = np.asarray(upper, dtype=float)
    below = np.clip(low + margin - position, 0.0, None)
    above = np.clip(position - (high - margin), 0.0, None)
    return float(np.sum(below) + np.sum(above))


def combine(
    terms: Mapping[str, float],
    dt: float,
    weights: Mapping[str, float] | None = None,
) -> tuple[float, dict[str, float]]:
    """Clip each weighted term to +-1, scale by dt once, and sum."""
    weights = weights or DEFAULT_WEIGHTS
    contribution: dict[str, float] = {}
    total = 0.0
    for name, value in terms.items():
        weight = float(weights.get(name, 0.0))
        clipped = float(np.clip(weight * float(value), -1.0, 1.0))
        contribution[name] = clipped * dt
        total += contribution[name]
    return total, contribution

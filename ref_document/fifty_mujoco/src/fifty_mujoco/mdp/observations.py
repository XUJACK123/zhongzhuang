"""Observation assembly (doc sections 5.2, 5.3, 12.7).

The actor sees 25 numbers per frame; five frames are stacked into 125.  The
critic sees 141 privileged numbers (149 with two gas springs) and the encoder
latent is appended by the network, so the critic input is 144 or 152.
"""

from __future__ import annotations

import numpy as np

from ..contracts import names

#: cmd scaling frozen to the teacher configuration (doc 5.2).
COMMAND_SCALE = np.array([3.0, 0.25, 5.0])


def proprio(
    ang_vel_body: np.ndarray,
    projected_gravity: np.ndarray,
    command: np.ndarray,
    joint_position: np.ndarray,
    joint_velocity: np.ndarray,
    last_action: np.ndarray,
    default_angles: np.ndarray,
    *,
    ang_vel_scale: float = 0.25,
    joint_velocity_scale: float = 0.05,
) -> np.ndarray:
    """25-dim actor observation, in the frozen order."""
    leg_slice = list(names.POLICY_LEG_INDICES)
    obs = np.zeros(names.ACTOR_OBS_DIM)
    obs[0:3] = np.asarray(ang_vel_body, dtype=float) * ang_vel_scale
    obs[3:6] = np.asarray(projected_gravity, dtype=float)
    obs[6:9] = np.asarray(command, dtype=float)[:3] * COMMAND_SCALE
    obs[9:13] = np.asarray(joint_position, dtype=float)[leg_slice] - np.asarray(default_angles, dtype=float)[leg_slice]
    obs[13:19] = np.asarray(joint_velocity, dtype=float) * joint_velocity_scale
    obs[19:25] = np.asarray(last_action, dtype=float)
    return obs


def privileged(
    base_lin_vel: np.ndarray,
    obs25: np.ndarray,
    last_action: np.ndarray,
    previous_action: np.ndarray,
    joint_acceleration: np.ndarray,
    heights77: np.ndarray,
    motor_torque: np.ndarray,
    mass_bias: float,
    com_bias: np.ndarray,
    default_angle_bias: np.ndarray,
    friction: float,
    restitution: float,
    *,
    lin_vel_scale: float = 2.0,
    acceleration_scale: float = 0.0025,
    torque_scale: float = 0.05,
) -> np.ndarray:
    """141-dim privileged observation (the base layout the teacher was trained on)."""
    parts = [
        np.asarray(base_lin_vel, dtype=float) * lin_vel_scale,  # 3
        np.asarray(obs25, dtype=float),  # 25
        np.asarray(last_action, dtype=float),  # 6  a(t-1)
        np.asarray(previous_action, dtype=float),  # 6  a(t-2)
        np.asarray(joint_acceleration, dtype=float) * acceleration_scale,  # 6
        np.asarray(heights77, dtype=float),  # 77
        np.asarray(motor_torque, dtype=float) * torque_scale,  # 6
        np.array([float(mass_bias)]),  # 1
        np.asarray(com_bias, dtype=float),  # 3
        np.asarray(default_angle_bias, dtype=float),  # 6
        np.array([float(friction)]),  # 1
        np.array([float(restitution)]),  # 1
    ]
    result = np.concatenate(parts)
    if result.size != names.BASE_PRIVILEGED_DIM:
        raise ValueError(f"privileged obs is {result.size}, expected {names.BASE_PRIVILEGED_DIM}")
    return result


def spring_features(
    lengths: np.ndarray,
    length_rates: np.ndarray,
    reference_forces: np.ndarray,
    actual_forces: np.ndarray,
    *,
    length_scale: float = 5.0,
    force_scale: float = 1.0 / 300.0,
) -> np.ndarray:
    """8 privileged spring numbers, right then left (doc 12.7)."""
    features = np.concatenate(
        [
            np.asarray(lengths, dtype=float) * length_scale,
            np.asarray(length_rates, dtype=float),
            np.asarray(reference_forces, dtype=float) * force_scale,
            np.asarray(actual_forces, dtype=float) * force_scale,
        ]
    )
    if features.size != names.GAS_SPRING_FEATURES_PER_SPRING * len(lengths):
        raise ValueError("spring feature count mismatch")
    return features


def flatten_history(history: np.ndarray) -> np.ndarray:
    """``[5, 25]`` oldest->newest into a 125 vector."""
    history = np.asarray(history, dtype=float)
    if history.shape[-2:] != (names.HISTORY_LENGTH, names.ACTOR_OBS_DIM):
        raise ValueError(f"history shape {history.shape} is not (...,5,25)")
    return history.reshape(*history.shape[:-2], names.HISTORY_DIM)


def push_history(history: np.ndarray, observation: np.ndarray, env_ids: np.ndarray | None = None) -> None:
    """Shift left and append, in place.  ``history`` is ``[N,5,25]``."""
    if env_ids is None:
        history[:, :-1] = history[:, 1:]
        history[:, -1] = observation
        return
    history[env_ids, :-1] = history[env_ids, 1:]
    history[env_ids, -1] = np.asarray(observation)[env_ids]


def fill_history(history: np.ndarray, env_ids: np.ndarray, observation: np.ndarray) -> None:
    history[env_ids] = observation[env_ids][:, None, :]

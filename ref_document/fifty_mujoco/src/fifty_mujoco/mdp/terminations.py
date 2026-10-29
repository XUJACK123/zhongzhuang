"""Termination and timeout logic (doc section 6.2).

Failure and timeout are kept apart: a timeout bootstraps with the value of the
*terminal* observation, a real failure never does.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class TerminationConfig:
    max_tilt_deg: float = 30.0
    tilt_grace_s: float = 0.2
    height_ratio: float = 0.6
    height_grace_s: float = 0.2
    episode_length_s: float = 20.0

    @property
    def tilt_cos(self) -> float:
        return float(np.cos(np.radians(self.max_tilt_deg)))


def tilt_angle_deg(rotation: np.ndarray) -> float:
    """Angle between the body +Z and the world +Z."""
    up = float(np.clip(np.asarray(rotation, dtype=float)[2, 2], -1.0, 1.0))
    return float(np.degrees(np.arccos(up)))


def orientation_failed(rotation: np.ndarray, config: TerminationConfig) -> bool:
    return np.asarray(rotation, dtype=float)[2, 2] < config.tilt_cos


def height_failed(root_height: float, nominal_height: float, config: TerminationConfig) -> bool:
    return root_height < config.height_ratio * nominal_height


def nonfinite_failed(**values: np.ndarray) -> bool:
    for value in values.values():
        if not np.all(np.isfinite(value)):
            return True
    return False


def illegal_contacts(model: mujoco.MjModel, data: mujoco.MjData, wheel_body_ids: set[int]) -> list[str]:
    """Names of robot bodies touching the floor (wheels excluded)."""
    names: list[str] = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        body_a = int(model.geom_bodyid[contact.geom1])
        body_b = int(model.geom_bodyid[contact.geom2])
        if body_a == 0 and body_b == 0:
            continue
        other = body_b if body_a == 0 else body_a
        if other == 0 or other in wheel_body_ids:
            continue
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, other)
        if body_name and body_name not in names:
            names.append(body_name)
    return names

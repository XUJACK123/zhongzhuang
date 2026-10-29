"""Command sampling (doc section 5.2 / 7.5).

The third command is a *height* command, never a lateral velocity.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class CommandRanges:
    lin_vel_x: tuple[float, float] = (-0.3, 0.3)
    ang_vel_z: tuple[float, float] = (-0.3, 0.3)
    height_offset: tuple[float, float] = (-0.03, 0.01)
    standing_fraction: float = 0.1
    resample_time_s: float = 5.0


@dataclass
class Commands:
    values: np.ndarray  # (N, 3): vx, yaw_rate, height
    resample_timer: np.ndarray

    @property
    def num_envs(self) -> int:
        return int(self.values.shape[0])


def sample_commands(
    num_envs: int,
    nominal_height: float,
    ranges: CommandRanges,
    rng: np.random.Generator,
) -> Commands:
    values = np.zeros((num_envs, 3))
    values[:, 0] = rng.uniform(*ranges.lin_vel_x, size=num_envs)
    values[:, 1] = rng.uniform(*ranges.ang_vel_z, size=num_envs)
    values[:, 2] = nominal_height + rng.uniform(*ranges.height_offset, size=num_envs)

    standing = rng.random(num_envs) < ranges.standing_fraction
    values[standing, 0] = 0.0
    values[standing, 1] = 0.0

    return Commands(values=values, resample_timer=np.full(num_envs, ranges.resample_time_s))


def set_commands(commands: Commands, env_ids: np.ndarray, values: np.ndarray) -> None:
    commands.values[env_ids] = values


def fixed_command(num_envs: int, vx: float, wz: float, height: float) -> Commands:
    values = np.tile(np.array([vx, wz, height], dtype=float), (num_envs, 1))
    return Commands(values=values, resample_timer=np.full(num_envs, 1e9))

"""The chassis must be upright: the recorded G1 base rotation left it inverted."""

from __future__ import annotations

import numpy as np
import pytest

from fifty_mujoco.assets.kinematics import link_poses
from fifty_mujoco.control.spawn import load_spawn_spec, resolve_spawn_height
from fifty_mujoco.envs.cpu_env import FiftyCpuEnv


@pytest.fixture(scope="module")
def env(built_assets, config):
    _, paths = built_assets
    return FiftyCpuEnv(paths.train, config)


def _chassis_up(config) -> np.ndarray:
    """Base mesh +Z expressed in the training frame after all corrections."""
    correction = config.mesh_correction("base_link")
    total = correction.R @ config.base_transform.R if correction is not None else config.base_transform.R
    return total @ np.array([0.0, 0.0, 1.0])


def test_chassis_up_is_world_up(config):
    up = _chassis_up(config)
    # The source CAD is authored with +Z up; after the correction the chassis
    # up axis must be the training +Z (dot = 1), not flipped (dot = -1).
    assert up @ np.array([0.0, 0.0, 1.0]) > 0.999999


def test_chassis_faces_the_leg_forward_direction(config):
    """The chassis front (source +Y) must point along the legs' forward (+X)."""
    correction = config.mesh_correction("base_link")
    total = correction.R @ config.base_transform.R if correction is not None else config.base_transform.R
    forward = total @ np.array([0.0, 1.0, 0.0])
    assert forward @ np.array([1.0, 0.0, 0.0]) > 0.999999


def test_base_top_marker_is_above_the_hip_axis(built_assets, config):
    corrected, _ = built_assets
    poses = link_poses(corrected.robot, config.default_angles)
    top = np.asarray(corrected.report["base_geometry"]["top_point_local_m"], dtype=float)
    assert poses["base_link"].apply(top)[2] > poses["base_link"].t[2]


def test_base_com_lies_inside_the_chassis(built_assets):
    corrected, _ = built_assets
    com = corrected.robot.links["base_link"].inertial.com
    low = np.asarray(corrected.report["base_geometry"]["aabb_min_m"], dtype=float)
    high = np.asarray(corrected.report["base_geometry"]["aabb_max_m"], dtype=float)
    assert np.all(com >= low) and np.all(com <= high)
    # The corrected chassis is upright: its highest CAD vertex is above the COM.
    top = np.asarray(corrected.report["base_geometry"]["top_point_local_m"], dtype=float)
    assert top[2] > com[2]


def test_spawn_keeps_the_robot_upright(built_assets, config, env):
    spec = load_spawn_spec(config)
    resolve_spawn_height(env, spec)
    rotation = np.asarray(env.data.xmat[env.body_ids["base_link"]], dtype=float).reshape(3, 3)
    assert rotation[2, 2] > 0.999999
    # Both wheels below the base origin and touching the ground plane.
    for wheel in ("right_wheel", "left_wheel"):
        assert env.data.xpos[env.body_ids[wheel]][2] < env.data.qpos[2]

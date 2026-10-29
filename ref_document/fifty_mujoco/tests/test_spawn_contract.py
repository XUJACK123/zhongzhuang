"""P2: upright spawn contract, name mapping and interface dimensions."""

from __future__ import annotations

import numpy as np
import pytest

from fifty_mujoco.contracts import names
from fifty_mujoco.contracts.names import MODEL_JOINT_ORDER, privileged_dim
from fifty_mujoco.control.spawn import audit_spawn, load_spawn_spec, resolve_spawn_height
from fifty_mujoco.envs.cpu_env import FiftyCpuEnv


@pytest.fixture(scope="module")
def env(built_assets, config):
    _, paths = built_assets
    return FiftyCpuEnv(paths.train, config)


def test_spawn_report_passes(env, config):
    report = audit_spawn(env, config, load_spawn_spec(config))
    for name, check in report["checks"].items():
        assert check["passed"], f"{name} failed: {check}"
    assert report["status"] == "passed"


def test_root_quaternion_is_identity_and_upright(env, config):
    spec = load_spawn_spec(config)
    resolve_spawn_height(env, spec)
    state = env.state()
    assert np.allclose(state["root_quat_wxyz"], [1.0, 0.0, 0.0, 0.0], atol=1e-12)
    assert np.isclose(state["root_quat_norm"], 1.0, atol=1e-12)


def test_wheels_clear_the_ground_by_ten_millimetres(env, config):
    spec = load_spawn_spec(config)
    _, bottoms = resolve_spawn_height(env, spec)
    assert set(bottoms) == {"right_wheel", "left_wheel"}
    for value in bottoms.values():
        assert abs(value - spec.wheel_clearance_m) <= 1e-9


def test_no_penetration_and_no_contact_at_spawn(env, config):
    spec = load_spawn_spec(config)
    resolve_spawn_height(env, spec)
    assert env.data.ncon == 0
    for geom_id in env.robot_geom_ids:
        lowest = None
        if int(env.model.geom_type[geom_id]) == 5:  # cylinder
            rotation = env.data.geom_xmat[geom_id].reshape(3, 3)
            axis = rotation[:, 2]
            lowest = (
                env.data.geom_xpos[geom_id][2]
                - env.model.geom_size[geom_id][1] * abs(axis[2])
                - env.model.geom_size[geom_id][0] * np.sqrt(max(0.0, 1.0 - axis[2] ** 2))
            )
        if lowest is not None:
            assert lowest >= -1e-9


def test_joint_name_mapping_is_explicit(env):
    for joint_name in MODEL_JOINT_ORDER:
        assert joint_name in env.joint_ids
        # qpos addresses are not assumed to be 7..12
        assert env.qpos_adr[joint_name] >= 7
        assert env.dof_adr[joint_name] >= 6


def test_actuator_count_is_not_policy_action_count(env):
    # Two tendon motors are added for the passive springs; the policy still has 6.
    assert env.model.nu == 6 + len(env.springs)
    assert env.model.ntendon == len(env.springs)
    assert names.NUM_ACTIONS == 6


def test_wheel_joints_are_unlimited(env):
    for name in names.WHEEL_JOINTS:
        joint_id = env.joint_ids[name]
        assert bool(env.model.jnt_limited[joint_id]) is False


def test_privileged_dimensions_follow_the_spring_count():
    assert privileged_dim(0) == (141, 144)
    assert privileged_dim(2) == (149, 152)


def test_contract_constants_match_the_frozen_document():
    assert names.ACTOR_OBS_DIM == 25
    assert names.HISTORY_DIM == 125
    assert names.NUM_ACTIONS == 6
    assert names.POLICY_LEG_INDICES == (0, 1, 3, 4)
    assert names.POLICY_WHEEL_INDICES == (2, 5)

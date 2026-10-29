"""P1: corrected assets and release gates (doc section 3.6)."""

from __future__ import annotations

import numpy as np

from fifty_mujoco.assets.kinematics import link_poses
from fifty_mujoco.math3d import inertia_is_physically_valid


def test_model_validation_passes(validation):
    for name, check in validation["checks"].items():
        assert check["passed"], f"{name} failed: {check}"
    assert validation["status"] == "passed"


def test_dimensions_match_the_documented_contract(validation, config):
    dimensions = validation["checks"]["model_dimensions"]
    assert dimensions["nq"] == 13
    assert dimensions["nv"] == 12
    # nu counts actuators: six joint motors plus one tendon motor per spring.
    assert dimensions["nu"] == 6 + config.spring_count
    assert dimensions["ntendon"] == config.spring_count
    assert dimensions["nbody"] == 8  # world + 7 links


def test_mass_matches_the_source_model(validation):
    total = validation["checks"]["total_mass"]["compiled_kg"]
    assert abs(total - 23.259630515567018) <= 1e-6


def test_spawn_height_matches_the_documented_value(built_assets):
    _, paths = built_assets
    assert abs(paths.spawn_z - 0.47475137216108476) <= 1e-9


def test_fk_parity_is_at_machine_precision(validation):
    train = validation["checks"]["fk_parity_train"]
    assert train["worst_position_error_m"] <= 1e-6
    assert train["worst_rotation_error_deg"] <= 1e-3


def test_wheel_geometry_is_centred_on_the_axle(validation):
    wheel = validation["checks"]["wheel_geometry"]
    assert wheel["passed"]
    for name, detail in wheel["per_wheel"].items():
        assert detail["visual_offset_m"] <= 1e-6, name
        assert detail["collision_offset_m"] <= 1e-6, name
        assert detail["joint_axis_dot_link_z"] >= 1.0 - 1e-9, name


def test_visual_and_train_models_share_physics(validation):
    parity = validation["checks"]["physical_parity_visual_vs_train"]
    assert parity["passed"]
    for name, detail in parity["bodies"].items():
        assert detail["mass_delta_kg"] <= 1e-9, name
        assert detail["ipos_delta_m"] <= 1e-9, name
        assert detail["inertia_delta"] <= 1e-9, name


def test_corrected_inertials_are_physically_valid_and_unrotated(built_assets):
    corrected, _ = built_assets
    for name, link in corrected.robot.links.items():
        if link.inertial is None:
            continue
        assert np.allclose(link.inertial.rpy, 0.0), f"{name}: inertial rpy must be zero"
        valid, reason = inertia_is_physically_valid(link.inertial.inertia)
        assert valid, f"{name}: {reason}"


def test_corrected_visual_origins_are_near_identity_for_moving_links(built_assets):
    corrected, _ = built_assets
    # The G1 defect moved wheel and shin meshes by 0.24-0.36 m; the corrected
    # model must not carry any such offset.
    for name in ("right_wheel", "left_wheel", "r1_link", "l1_link"):
        offset = float(np.linalg.norm(corrected.robot.links[name].visuals[0].xyz))
        assert offset <= 1e-6, f"{name}: residual visual offset {offset}"


def test_default_pose_is_above_the_ground(built_assets, config):
    corrected, _ = built_assets
    poses = link_poses(corrected.robot, config.default_angles)
    for name in ("right_wheel", "left_wheel"):
        bottom = poses[name].t[2] - config.wheel_radius
        assert bottom < 0.0  # wheels are below the base origin
    # Both wheels at the same height (symmetric stance).
    assert abs(poses["right_wheel"].t[2] - poses["left_wheel"].t[2]) <= 1e-9


def test_gas_spring_is_present_and_declared_uncalibrated(validation, config):
    assert config.gas_spring_configured is True
    assert config.spring_count == 2
    spring = validation["checks"]["gas_spring"]
    assert spring["tendon_count"] == 2
    assert spring["status"] == f"generated_{config.gas_spring_profile}"
    # Anchors are transferred from the Fudan model and the stroke is unknown,
    # so the reference profile must not claim calibration.
    assert spring["stroke_verified"] is False
    assert config.gas_spring_profile == "reference_300n"

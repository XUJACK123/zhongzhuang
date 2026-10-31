"""Joint convention contract (progress.md §23.4 / task B).

Three things used to be prose, and prose cannot fail:

* which axis each joint turns about, and in which sense ``+`` means more,
* how the left channels relate to the right ones,
* which standing pose the reference numbers describe.

Every assertion here either reads the model file, the policy contract or the
confirmed closed chain - nothing is asserted from memory.
"""

from __future__ import annotations

import math

import pytest
import yaml

from fifty_mujoco.mechanism import load_mechanism
from fifty_mujoco.mechanism.bridge import (
    MIN_JACOBIAN_DET,
    crank_roots,
    jacobian_determinant,
    leg_pose,
    leg_pose_from_serial,
    mechanism_from_serial,
    shin_angle,
    wheel_jacobian,
)
from fifty_mujoco.mechanism.conventions import (
    AXIS_TOLERANCE,
    JointConventionsError,
    load_conventions,
    mjcf_joint_axes,
    parse_conventions,
    reference_pose,
)
from fifty_mujoco.mechanism.leg import TwoLinkLeg
from fifty_mujoco.mechanism.planar import NoSolutionError
from fifty_mujoco.mechanism.transmission import ThighCrankTransmission
from fifty_mujoco.paths import configs_dir


@pytest.fixture(scope="module")
def conventions():
    return load_conventions()


@pytest.fixture(scope="module")
def mechanism():
    return load_mechanism()


@pytest.fixture(scope="module")
def transmission(mechanism):
    return ThighCrankTransmission.from_mechanism(mechanism)


def _raw(path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


# ------------------------------------------------------------------ the table
def test_the_declared_axes_are_the_ones_the_model_uses(conventions):
    """A typo in the table would otherwise only show up as a robot spinning."""
    verification = conventions.verify()
    assert verification["status"] == "passed", verification["problems"]
    assert set(verification["axes"]) == set(conventions.names)


def test_every_channel_agrees_with_the_policy_contract_and_robot_yaml(conventions):
    robot = _raw(configs_dir() / "robot.yaml")
    contract = _raw(configs_dir() / "policy_contract.yaml")
    verification = conventions.verify(axes=mjcf_joint_axes(), robot=robot, contract=contract)
    assert verification["status"] == "passed", verification["problems"]


def test_a_wrong_axis_is_reported_not_ignored(conventions):
    axes = dict(mjcf_joint_axes())
    axes["rf0_Joint"] = (0.0, -1.0, 0.0)
    verification = conventions.verify(axes=axes)
    assert verification["status"] == "failed"
    assert any("rf0_Joint" in problem for problem in verification["problems"])


def test_a_wrong_zero_is_reported_not_ignored(conventions):
    robot = _raw(configs_dir() / "robot.yaml")
    robot["joints"]["default_angles_rad"]["rf1_Joint"] += 1e-3
    verification = conventions.verify(robot=robot)
    assert verification["status"] == "failed"
    assert any("rf1_Joint" in problem for problem in verification["problems"])


def test_the_wheels_share_an_axis_while_the_leg_joints_mirror(conventions):
    """Both wheels roll about base +Y; the leg joints are mirrored about it."""
    axes = mjcf_joint_axes()
    assert axes["r_wheel_Joint"] == pytest.approx((0.0, 1.0, 0.0), abs=AXIS_TOLERANCE)
    assert axes["l_wheel_Joint"] == pytest.approx((0.0, 1.0, 0.0), abs=AXIS_TOLERANCE)
    assert axes["rf0_Joint"] == pytest.approx((0.0, 1.0, 0.0), abs=AXIS_TOLERANCE)
    assert axes["lf0_Joint"] == pytest.approx((0.0, -1.0, 0.0), abs=AXIS_TOLERANCE)
    # the declaration must not have been "fixed" to match a guess
    assert conventions.channel("l_wheel_Joint").axis_base == (0.0, 1.0, 0.0)
    assert conventions.channel("rf0_Joint").sign == 1.0
    assert conventions.channel("lf0_Joint").sign == -1.0


def test_mirroring_a_symmetric_pose_is_the_identity_on_the_right_side(conventions):
    right = (2.0, -2.5, 0.4, 0.0, 0.0, 0.0)
    mirrored = conventions.mirror_pose(right)
    assert mirrored[:3] == pytest.approx(right[:3])
    assert mirrored[3] == pytest.approx(-2.0)
    assert mirrored[4] == pytest.approx(2.5)
    # the wheel channel is not negated: both wheels turn the same way
    assert mirrored[5] == pytest.approx(0.4)
    assert conventions.symmetric(mirrored)


def test_left_and_right_round_trip_in_both_directions(conventions):
    for name in ("lf0_Joint", "lf1_Joint"):
        right_value = 0.37
        own = conventions.to_own_frame(name, right_value)
        assert conventions.to_right_frame(name, own) == pytest.approx(right_value)
    for name in ("rf0_Joint", "rf1_Joint", "r_wheel_Joint", "l_wheel_Joint"):
        assert conventions.to_right_frame(name, 1.5) == pytest.approx(1.5)


def test_a_zero_sign_is_rejected(conventions):
    data = _raw(configs_dir() / "joint_conventions.yaml")
    data["channels"][0]["sign"] = 0.0
    with pytest.raises(JointConventionsError, match="sign"):
        parse_conventions(data, configs_dir() / "joint_conventions.yaml")


def test_a_missing_reference_pose_is_rejected(conventions):
    data = _raw(configs_dir() / "joint_conventions.yaml")
    data["reference_pose"].pop("solution")
    with pytest.raises(JointConventionsError, match="reference_pose"):
        parse_conventions(data, configs_dir() / "joint_conventions.yaml")


def test_the_zero_is_an_asset_convention_not_a_hardware_zero(conventions):
    solution = conventions.reference.get("solution") or {}
    assert "not_a_hardware_zero" in conventions.reference
    # the reference pose must not be confused with the encoder zero
    assert solution["thigh_rad"] != pytest.approx(conventions.channel("rf0_Joint").zero_rad)


# ---------------------------------------------------------------- the reference
def test_the_reference_pose_puts_the_wheel_on_the_target_chord(conventions, mechanism):
    pose = reference_pose(conventions, mechanism)
    target = conventions.reference["target"]
    assert pose.chord_m == pytest.approx(target["chord_m"], abs=1e-9)
    assert pose.wheel_from_hip[0] == pytest.approx(
        target["wheel_offset_from_hip_m"], abs=1e-9
    )
    assert pose.wheel_from_hip[1] == pytest.approx(-target["chord_m"], abs=1e-9)


def test_the_reference_pose_is_inside_the_chain_and_not_singular(conventions, mechanism):
    pose = reference_pose(conventions, mechanism)
    assert abs(pose.jacobian_det) >= MIN_JACOBIAN_DET
    assert pose.jacobian_det > 0.0


def test_the_stored_reference_numbers_are_the_recomputed_ones(conventions, mechanism):
    pose = reference_pose(conventions, mechanism)
    stored = conventions.reference_channel_values()
    assert pose.thigh_angle == pytest.approx(stored["thigh_rad"], abs=1e-9)
    assert pose.shin_angle == pytest.approx(stored["shin_rad"], abs=1e-9)
    assert pose.crank_angle == pytest.approx(stored["crank_rad"], abs=1e-9)
    stored_solution = conventions.reference["solution"]
    assert pose.jacobian_det == pytest.approx(stored_solution["jacobian_det"], abs=1e-9)


def test_the_reference_pose_is_not_the_stowed_cad_pose(conventions, transmission):
    """The CAD pose is folded up; using it as the working pose is the classic bug."""
    stored = conventions.reference_channel_values()
    assert abs(stored["thigh_rad"] - transmission.thigh_angle_cad) > 0.5
    assert abs(stored["crank_rad"] - transmission.crank_angle_cad) > 0.5


# ------------------------------------------------------------- the bridge (B)
def test_the_bridge_reproduces_the_serial_leg_at_the_reference(conventions, mechanism):
    pose = reference_pose(conventions, mechanism)
    leg = TwoLinkLeg(0.21, 0.25)
    thigh, shin = leg.inverse((0.0, 0.0), pose.wheel_from_hip, branch=1)
    assert thigh == pytest.approx(pose.thigh_angle, abs=1e-9)
    assert shin == pytest.approx(pose.shin_angle, abs=1e-9)


def test_serial_to_mechanism_round_trips(transmission):
    for thigh_deg in (-150.0, -121.19, -100.0, -90.0):
        for shin_deg in (-100.0, -64.21, -40.0):
            thigh, shin = math.radians(thigh_deg), math.radians(shin_deg)
            try:
                crank = mechanism_from_serial(transmission, thigh, shin)
            except NoSolutionError:
                continue
            assert shin_angle(transmission, thigh, crank) == pytest.approx(
                shin, abs=1e-9
            )


def test_mechanism_to_serial_then_back_returns_the_same_crank(transmission):
    thigh = math.radians(-130.0)
    crank = math.radians(-55.0)
    _, shin = (thigh, shin_angle(transmission, thigh, crank))
    recovered = mechanism_from_serial(transmission, thigh, shin)
    assert recovered == pytest.approx(crank, abs=1e-9)


def test_a_pose_outside_the_feasible_set_raises_instead_of_clamping(transmission):
    # (thigh, shin) = (-90, -100) deg has no crank that closes the chain: the scan
    # only finds a feasibility-edge artefact there, which must not be returned.
    with pytest.raises(NoSolutionError):
        mechanism_from_serial(transmission, math.radians(-90.0), math.radians(-100.0))


def test_singular_roots_are_reported_and_not_selected(transmission):
    thigh = math.radians(-121.19)
    shin = math.radians(-64.214)
    roots = crank_roots(transmission, thigh, shin)
    assert roots, "the standing thigh/shin pair must have at least one root"
    assert any(not root.usable for root in roots), "the degenerate root at crank = thigh"
    chosen = mechanism_from_serial(transmission, thigh, shin)
    assert abs(jacobian_determinant(transmission, thigh, chosen)) >= MIN_JACOBIAN_DET


# --------------------------------------------------------------- the Jacobian
def test_the_jacobian_matches_a_finite_difference_step(conventions, mechanism):
    pose = reference_pose(conventions, mechanism)
    chain = ThighCrankTransmission.from_mechanism(mechanism)
    step = 1e-5
    plus = leg_pose(chain, pose.thigh_angle + 0, pose.crank_angle + step)
    minus = leg_pose(chain, pose.thigh_angle + 0, pose.crank_angle - step)
    columns = wheel_jacobian(chain, pose.thigh_angle, pose.crank_angle)
    numeric = (
        (plus.wheel_from_hip[0] - minus.wheel_from_hip[0]) / (2 * step),
        (plus.wheel_from_hip[1] - minus.wheel_from_hip[1]) / (2 * step),
    )
    assert columns[1] == pytest.approx(numeric, abs=1e-5)


def test_the_jacobian_maps_a_small_joint_step_to_the_right_wheel_motion(transmission):
    thigh = math.radians(-121.19)
    crank = math.radians(-58.81)
    base = transmission.forward(thigh, crank)["WHEEL"]
    delta = (1e-4, -2e-4)
    shifted = transmission.forward(thigh + delta[0], crank + delta[1])["WHEEL"]
    first, second = wheel_jacobian(transmission, thigh, crank)
    predicted = (
        first[0] * delta[0] + second[0] * delta[1],
        first[1] * delta[0] + second[1] * delta[1],
    )
    assert predicted[0] == pytest.approx(shifted[0] - base[0], abs=5e-8)
    assert predicted[1] == pytest.approx(shifted[1] - base[1], abs=5e-8)


def test_leg_pose_from_serial_equals_the_explicit_solve(transmission):
    thigh = math.radians(-121.19)
    shin = math.radians(-64.214)
    direct = leg_pose_from_serial(transmission, thigh, shin)
    crank = mechanism_from_serial(transmission, thigh, shin)
    explicit = leg_pose(transmission, thigh, crank)
    assert direct.crank_angle == pytest.approx(explicit.crank_angle)
    assert direct.chord_m == pytest.approx(explicit.chord_m)


def test_serial_candidate_and_runtime_defaults_are_distinct_poses(conventions):
    from fifty_mujoco.config import load_yaml
    candidate = conventions.reference["serial_reference_candidate"]
    solution = conventions.reference["solution"]
    q0, q1 = candidate["rf0_Joint"], candidate["rf1_Joint"]
    thigh, shin = -q0, -(q0 + math.pi / 2.0 + q1)
    assert thigh == pytest.approx(solution["thigh_rad"], abs=1e-9)
    assert shin == pytest.approx(solution["shin_rad"], abs=1e-9)
    assert 0.21 * math.cos(thigh) + 0.25 * math.cos(shin) == pytest.approx(0.0, abs=1e-9)
    defaults = load_yaml(configs_dir() / "robot.yaml")["joints"]["default_angles_rad"]
    old_t = -defaults["rf0_Joint"]
    old_s = -(defaults["rf0_Joint"] + math.pi / 2.0 + defaults["rf1_Joint"])
    expected = conventions.reference["runtime_default_pose"]
    assert 0.21 * math.cos(old_t) + 0.25 * math.cos(old_s) == pytest.approx(
        expected["wheel_forward_of_hip_m"], abs=1e-9
    )
    assert expected["wheel_forward_of_hip_m"] > 0.02


def test_serial_joint_angles_reproduce_the_reference_wheel_below_the_hip(conventions):
    """Absolute link angles -> serial hinge values, checked by forward kinematics.

    No MuJoCo: ``mjcf_body_positions`` composes the model's own body offsets and
    hinge axes, so this is a static statement about the file.  It is also the
    guard against mixing absolute link angles with joint angles.
    """
    from fifty_mujoco.mechanism.conventions import mjcf_body_positions
    from fifty_mujoco.mechanism.resolve import serial_joint_angles

    solution = conventions.reference["solution"]
    qpos = serial_joint_angles(solution["thigh_rad"], solution["shin_rad"])
    for side in ("right", "left"):
        positions = mjcf_body_positions(qpos)
        hip = positions["r0_link" if side == "right" else "l0_link"]
        wheel = positions["right_wheel" if side == "right" else "left_wheel"]
        # the reference pose has the wheel directly below the hip
        assert wheel[0] - hip[0] == pytest.approx(0.0, abs=1e-9)
        assert hip[2] - wheel[2] == pytest.approx(
            conventions.reference["target"]["chord_m"], abs=1e-9
        )
    candidate = conventions.reference["serial_reference_candidate"]
    for name, value in qpos.items():
        assert candidate[name] == pytest.approx(value, abs=1e-9)


def test_the_runtime_default_pose_does_not_put_the_wheel_below_the_hip():
    """The old serial default is 22.865 mm ahead: it must not be called vertical."""
    from fifty_mujoco.mechanism.resolve import serial_default_pose

    pose = serial_default_pose()
    for side in ("right", "left"):
        assert pose[side]["wheel_forward_of_hip_mm"] == pytest.approx(22.865346, abs=1e-6)
        assert pose[side]["wheel_below_hip_mm"] == pytest.approx(404.751372, abs=1e-6)
        assert pose[side]["chord_mm"] > pose[side]["wheel_below_hip_mm"]

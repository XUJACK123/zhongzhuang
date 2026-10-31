"""Absolute link angles vs hinge values: the conversion must stay honest.

The planar mechanism speaks in absolute link angles; MuJoCo speaks in hinge values
measured from the CAD assembly pose.  Mixing the two produces a pose that looks
plausible and is wrong, so every number here is recomputed from geometry rather
than retyped.
"""

from __future__ import annotations

import math

import pytest

from fifty_mujoco.mechanism.closed_pose import (
    AXIS_SIGN,
    BODIES,
    DRIVEN,
    JOINT_NAMES,
    SIDES,
    assembly_pose,
    body_angles,
    reference_pose,
    reference_pose_from_angles,
    wrap,
)
from fifty_mujoco.mechanism.loader import load_mechanism
from fifty_mujoco.mechanism.transmission import ThighCrankTransmission


@pytest.fixture(scope="module")
def mechanism():
    return load_mechanism()


@pytest.fixture(scope="module")
def reference(mechanism):
    return reference_pose(mechanism)


def test_the_assembly_pose_is_qpos_zero(mechanism):
    pose = assembly_pose(mechanism)
    for body, value in pose.qpos.items():
        assert value == pytest.approx(0.0, abs=1e-15), body


def test_the_reference_pose_puts_the_wheel_directly_below_the_hip(mechanism, reference):
    transmission = ThighCrankTransmission.from_mechanism(mechanism)
    nodes = transmission.forward(-2.1151626422852363, -1.02643001134495)
    assert nodes["WHEEL"][0] - nodes["HIP"][0] == pytest.approx(0.0, abs=1e-12)
    assert nodes["HIP"][1] - nodes["WHEEL"][1] == pytest.approx(0.404751372161085, abs=1e-12)
    assert reference.chord_m == pytest.approx(0.404751372161085, abs=1e-12)


def test_the_reference_angles_are_the_planar_convention(reference):
    """thigh / shin / crank are reported with the pairs the planar model uses."""
    assert math.degrees(reference.angles["thigh"]) == pytest.approx(-121.189892, abs=5e-6)
    assert math.degrees(reference.angles["shin"]) == pytest.approx(-64.213693, abs=5e-6)
    assert math.degrees(reference.angles["crank"]) == pytest.approx(-58.810108, abs=5e-6)


def test_the_hinge_values_are_not_the_absolute_angles(reference):
    """A concrete guard: a hinge is a relative rotation, signed by the +Y axis."""
    assert reference.angles["thigh"] == pytest.approx(-2.1151626422852363, abs=1e-12)
    # +Y turns the planar angle negative, and the thigh's parent is the trunk,
    # so the value is the *negated* angle change since the CAD pose.
    assert reference.qpos["thigh"] == pytest.approx(-1.738502588860534, abs=1e-12)
    # the old bug produced the positive absolute difference; the sign flip is
    # exactly what left the loops open by 0.24 m
    assert reference.qpos["thigh"] != pytest.approx(1.738502588860534, abs=1e-9)


def test_every_body_gets_a_hinge_value(reference):
    assert set(reference.qpos) == {body for body, _, _ in BODIES}
    assert reference.qpos["wheel"] == 0.0


def _angles_from_qpos(qpos, zero):
    """Rebuild absolute planar angles the way MuJoCo composes the hinges.

    ``delta_body = delta_parent - q_body`` for a +Y hinge; the trunk has no hinge.
    """
    from fifty_mujoco.mechanism.closed_pose import TREE

    rebuilt: dict[str, float] = {}
    delta: dict[str, float] = {}
    for body, parent in TREE:
        if body == "wheel":
            continue
        parent_delta = delta.get(parent, 0.0)
        delta[body] = parent_delta - qpos[body]
        rebuilt[body] = zero[body] + delta[body]
    return rebuilt


def test_the_loops_are_consistent_in_the_reference_pose(mechanism, reference):
    """Re-deriving the angles from the hinge values must give the same pose."""
    transmission = ThighCrankTransmission.from_mechanism(mechanism)
    zero = assembly_pose(mechanism).angles
    angles = _angles_from_qpos(reference.qpos, zero)
    for body, target in reference.angles.items():
        assert wrap(angles[body] - target) == pytest.approx(0.0, abs=1e-12), body
    rebuilt = transmission.forward(angles["thigh"], angles["crank"])
    expected = transmission.forward(-2.1151626422852363, -1.02643001134495)
    for node, point in expected.items():
        assert math.dist(rebuilt[node], point) == pytest.approx(0.0, abs=1e-9), node


def test_a_different_driver_pair_gives_a_different_pose(mechanism):
    other = reference_pose_from_angles(mechanism, -2.0, -1.0)
    reference = reference_pose(mechanism)
    assert other.qpos["thigh"] != pytest.approx(reference.qpos["thigh"])
    assert other.chord_m != pytest.approx(reference.chord_m)


def test_the_left_side_is_the_mirrored_value(reference):
    right = reference.joint_values("right")
    left = reference.joint_values("left")
    assert right["r_wheel_Joint"] == pytest.approx(0.0)
    assert left["l_wheel_Joint"] == pytest.approx(0.0)
    for name, value in right.items():
        twin = name.replace("r_", "l_", 1) if name.startswith("r_") else name.replace("rf", "lf", 1)
        if twin.endswith("wheel_Joint"):
            continue
        # leg hinges turn about -Y on the left, so the value is negated
        assert left[twin] == pytest.approx(-value), (name, twin)


def test_wrap_is_signed_and_bounded():
    assert wrap(math.pi) == pytest.approx(math.pi)
    assert wrap(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


def test_the_stored_config_values_match_the_computation(reference):
    """`joint_conventions.yaml: reference_pose.closed_qpos` must not drift."""
    from fifty_mujoco.config import load_yaml
    from fifty_mujoco.paths import configs_dir

    stored = load_yaml(configs_dir() / "joint_conventions.yaml")["reference_pose"]["closed_qpos"][
        "right"
    ]
    computed = reference.joint_values("right")
    assert set(stored) == set(computed)
    for name, value in computed.items():
        assert stored[name] == pytest.approx(value, abs=1e-12), name
    solution = load_yaml(configs_dir() / "joint_conventions.yaml")["reference_pose"]["solution"]
    assert solution["thigh_rad"] == pytest.approx(reference.angles["thigh"], abs=1e-12)
    assert solution["crank_rad"] == pytest.approx(reference.angles["crank"], abs=1e-12)


def test_the_switch_to_the_reference_pose_is_written_down_but_not_applied():
    """The pose is validated; moving the runtime defaults is a separate, wider change.

    ``robot.yaml`` still holds the older defaults because several regression
    baselines (asset build, corrected-URDF roundtrip, Fudan anchor offsets) were
    recorded at that pose.  The conversion is ready and tested, so the switch is a
    data change plus a re-baseline, not new maths.
    """
    from fifty_mujoco.config import load_configs
    from fifty_mujoco.mechanism.resolve import serial_joint_angles

    target = serial_joint_angles(-2.1151626422852363, -1.120740359909)
    config = load_configs()
    assert config.default_angles["rf0_Joint"] != pytest.approx(target["rf0_Joint"])
    # the target values are stored next to the reference pose, ready to apply
    from fifty_mujoco.config import load_yaml
    from fifty_mujoco.paths import configs_dir

    candidate = load_yaml(configs_dir() / "joint_conventions.yaml")["reference_pose"][
        "serial_reference_candidate"
    ]
    for name, value in target.items():
        assert candidate[name] == pytest.approx(value, abs=1e-9), name


def test_the_old_defaults_still_reproduce_the_withdrawn_forward_offset(mechanism):
    """History: the previous defaults put the wheel 22.865 mm ahead. Keep it reproducible."""
    from fifty_mujoco.mechanism.conventions import mjcf_body_positions

    old = {
        "rf0_Joint": 2.0554215512416354,
        "rf1_Joint": -2.5592683579936746,
        "r_wheel_Joint": 0.0,
        "lf0_Joint": -2.055421551241647,
        "lf1_Joint": 2.559268357993675,
        "l_wheel_Joint": 0.0,
    }
    positions = mjcf_body_positions(old)
    hip, wheel = positions["r0_link"], positions["right_wheel"]
    assert (wheel[0] - hip[0]) * 1000.0 == pytest.approx(22.865346, abs=1e-5)
    assert (hip[2] - wheel[2]) * 1000.0 == pytest.approx(404.751372, abs=1e-5)

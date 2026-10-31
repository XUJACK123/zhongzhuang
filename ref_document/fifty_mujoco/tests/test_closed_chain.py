"""The candidate closed-chain model: everything checkable without MuJoCo.

``verify()`` compiles the model; this module covers the part that is a pure
function of the geometry, so a wrong number fails here even on a machine with no
simulation stack: XML structure, mass conservation, the inertia rotation, the
keyframe and the spring's lateral gap.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from fifty_mujoco.assets import build_closed_mjcf as builder
from fifty_mujoco.mechanism.closed_pose import JOINT_NAMES, SIDES, reference_pose
from fifty_mujoco.mechanism.loader import load_mechanism
from fifty_mujoco.paths import configs_dir


@pytest.fixture(scope="module")
def built():
    return builder.build_model()


@pytest.fixture(scope="module")
def xml(built) -> str:
    return ET.tostring(built[0]).decode()


@pytest.fixture(scope="module")
def authored(built) -> dict:
    return built[1]


def test_the_model_has_ground_collision_and_actuators(xml: str):
    assert 'name="floor"' in xml
    # base + 7 bodies per leg
    assert xml.count("<body ") == 15
    # 14 hinges
    assert xml.count('<joint name="') == 14
    # 6 joint motors + 2 spring tendons
    assert xml.count("<motor ") == 8
    assert xml.count("<spatial ") == 2
    assert xml.count("<connect ") == 4
    assert xml.count("<key ") == 1


def test_the_wheels_share_one_axis_direction(xml: str):
    """policy_contract says l_wheel is +Y; the leg hinges are the ones that mirror."""
    assert 'name="r_wheel_Joint" type="hinge" axis="0.0 1.0 0.0"' in xml
    assert 'name="l_wheel_Joint" type="hinge" axis="0.0 1.0 0.0"' in xml
    assert 'name="rf0_Joint" type="hinge" axis="0.0 1.0 0.0"' in xml
    assert 'name="lf0_Joint" type="hinge" axis="0.0 -1.0 0.0"' in xml


def test_the_wheels_are_not_limited(xml: str):
    for name in ("r_wheel_Joint", "l_wheel_Joint"):
        start = xml.index(f'<joint name="{name}"')
        end = xml.index("/>", start)
        assert 'limited="false"' in xml[start:end]
        assert "range=" not in xml[start:end]


def test_the_mass_is_conserved(authored: dict):
    """Loop bodies are paid for out of the thigh and shin; nothing is double counted."""
    serial = builder.load_serial_assets()
    for side, key in SIDES:
        expected = sum(
            serial[body].mass_kg
            for body in (
                ("r0_link", "r1_link", "right_wheel")
                if key == "r"
                else ("l0_link", "l1_link", "left_wheel")
            )
        )
        assert sum(authored["masses"][side].values()) == pytest.approx(expected, abs=1e-12)
        for name in builder.LOOP_BODIES:
            assert authored["masses"][side][name] == pytest.approx(builder.LOOP_BODY_MASS_KG)


def test_the_inherited_inertia_keeps_its_trace():
    """A rotation cannot change the inertia's trace: R·I·Rᵀ is a similarity."""
    serial = builder.load_serial_assets()
    for body in ("r0_link", "r1_link", "right_wheel", "left_wheel"):
        asset = serial[body]
        rotated = builder.rotate_inertia(asset.fullinertia, asset.quat_base)
        assert builder.inertia_trace(rotated) == pytest.approx(
            builder.inertia_trace(asset.fullinertia), abs=1e-12
        )


def test_the_inertia_rotation_is_a_proper_rotation():
    identical = builder.rotate_inertia((0.5, 1.5, 2.5, 0.1, -0.2, 0.3), (1.0, 0.0, 0.0, 0.0))
    assert identical == pytest.approx((0.5, 1.5, 2.5, 0.1, -0.2, 0.3), abs=1e-15)
    # 90 degrees about Z swaps ixx and iyy and negates ixy
    quarter = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    rotated = builder.rotate_inertia((0.5, 1.5, 2.5, 0.1, -0.2, 0.3), quarter)
    assert rotated[0] == pytest.approx(1.5, abs=1e-12)
    assert rotated[1] == pytest.approx(0.5, abs=1e-12)
    assert rotated[3] == pytest.approx(-0.1, abs=1e-12)


def test_the_collision_geoms_keep_their_real_size(xml: str):
    """The wheel must stay the real 0.06 m cylinder, not a placeholder sphere."""
    assert 'type="cylinder" size="0.06 ' in xml
    assert 'type="sphere"' not in xml


def test_the_spring_gap_is_the_measured_eleven_millimetres(authored: dict):
    for side in ("right", "left"):
        assert authored["spring"][side]["lateral_gap_mm"] == pytest.approx(11.0, abs=1e-9)
        assert authored["spring"][side]["force_n"] == pytest.approx(450.0)
        assert authored["spring"][side]["model"] == "constant_force_placeholder"


def test_the_keyframe_reproduces_the_reference_pose(built):
    pose = reference_pose(load_mechanism())
    key = built[0].find("keyframe/key")
    values = [float(v) for v in key.get("qpos").split()]
    assert values[:7] == pytest.approx(
        [0.0, 0.0, builder.STANDING_HEIGHT_M, 1.0, 0.0, 0.0, 0.0], abs=1e-12
    )
    for index, (side, key_) in enumerate(SIDES):
        per_side = pose.joint_values(side)
        expected = [per_side[JOINT_NAMES[body].format(key=key_)] for body in builder.QPOS_ORDER]
        chunk = values[7 + index * 7 : 7 + (index + 1) * 7]
        assert chunk == pytest.approx(expected, abs=1e-12), side


def test_the_keyframe_leaves_the_wheels_unrotated(built):
    values = [float(v) for v in built[0].find("keyframe/key").get("qpos").split()]
    for index in range(len(SIDES)):
        assert values[7 + index * 7 + 2] == 0.0


def test_the_leg_hinges_carry_the_side_sign(built):
    values = [float(v) for v in built[0].find("keyframe/key").get("qpos").split()]
    right = values[7:14]
    left = values[14:21]
    for index, body in enumerate(builder.QPOS_ORDER):
        if body == "wheel":
            assert left[index] == 0.0
        else:
            assert left[index] == pytest.approx(-right[index])


def test_the_loop_equalities_are_the_two_confirmed_loops(xml: str):
    assert 'site1="r_ternary_LINK_PIN_A"' in xml
    assert 'site2="r_thigh_LINK_PIN_A"' in xml
    assert 'site1="r_end_link_END_PIN_A"' in xml
    assert 'site2="r_shin_END_PIN_A"' in xml


def test_the_model_declares_what_is_provisional(authored: dict):
    ranges = authored["joint_ranges"]
    assert ranges["driven"] == [-builder.DRIVEN_RANGE_RAD, builder.DRIVEN_RANGE_RAD]
    assert "provisional" in ranges["note"]
    assert authored["spring"]["right"]["model"] == "constant_force_placeholder"


def test_the_two_coordinate_sets_are_kept_apart():
    """The serial defaults and the closed qpos describe one pose in two conventions."""
    text = (configs_dir() / "joint_conventions.yaml").read_text(encoding="utf-8")
    assert "closed_qpos" in text
    assert "must never be interchanged" in text

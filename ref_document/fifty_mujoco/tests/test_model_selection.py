"""Model selection: ``configs/models.yaml`` must be the only switch.

The closed-chain candidate exists now, and a check that passes on the serial
model says nothing about it.  These tests pin the pieces that make the choice
real: one registry, one body-name map, model-specific default angles, and the
passive-joint values read from the model's own keyframe.
"""

from __future__ import annotations

import pytest

from fifty_mujoco.assets.model_binding import (
    joint_qpos_addresses,
    keyframe_values,
    passive_joint_values,
)
from fifty_mujoco.config import load_configs

pytest.importorskip("mujoco")


@pytest.fixture()
def config():
    return load_configs()


def test_the_default_model_is_the_serial_one(config, monkeypatch):
    monkeypatch.delenv("FIFTY_MODEL", raising=False)
    assert config.model_name == "serial"
    assert config.model_xml().name == "fifty_train.xml"
    assert config.passive_joints == []
    assert not config.uses_keyframe_reset


def test_the_closed_model_is_selectable(config, monkeypatch):
    monkeypatch.setenv("FIFTY_MODEL", "closed")
    assert config.model_name == "closed"
    assert config.model_xml().name == "fifty_closed_train.xml"
    assert config.uses_keyframe_reset
    assert len(config.passive_joints) == 8
    assert config.model_status == "candidate"
    assert config.tendon_name("gas_right") == "gas_right_length"


def test_body_names_follow_the_selected_model(config, monkeypatch):
    monkeypatch.delenv("FIFTY_MODEL", raising=False)
    assert config.body_name("thigh_right") == "r0_link"
    monkeypatch.setenv("FIFTY_MODEL", "closed")
    assert config.body_name("thigh_right") == "r_thigh"
    assert config.body_name("wheel_left") == "l_wheel"


def test_closed_default_angles_are_the_reference_pose(config, monkeypatch):
    monkeypatch.setenv("FIFTY_MODEL", "closed")
    stored = config.joint_conventions["reference_pose"]["closed_qpos"]["right"]
    angles = config.default_angles
    for name, value in angles.items():
        source = name if name in stored else f"r{name[1:]}"
        sign = 1.0 if name in stored else -1.0
        assert value == pytest.approx(sign * float(stored[source]), abs=1e-12), name


def test_passive_joints_come_from_the_keyframe(config, monkeypatch):
    monkeypatch.setenv("FIFTY_MODEL", "closed")
    xml = config.model_xml()
    values = keyframe_values(xml, "reference")
    assert values is not None
    passive = passive_joint_values(xml, config.joint_order, "reference")
    assert set(passive) == set(config.passive_joints)
    assert all(value != 0.0 for value in passive.values())


def test_the_serial_model_has_no_passive_joints(config, monkeypatch):
    monkeypatch.delenv("FIFTY_MODEL", raising=False)
    assert passive_joint_values(config.model_xml(), config.joint_order) == {}


def test_qpos_addresses_match_mujoco(config, monkeypatch):
    """The stdlib parser must reproduce MuJoCo's own layout."""
    import mujoco

    for name in ("serial", "closed"):
        monkeypatch.setenv("FIFTY_MODEL", name)
        xml = config.model_xml()
        model = mujoco.MjModel.from_xml_path(str(xml))
        parsed = joint_qpos_addresses(xml)
        for joint, address in parsed.items():
            identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            assert identifier >= 0, joint
            assert address == int(model.jnt_qposadr[identifier]), (name, joint)


def test_unknown_model_is_rejected(config, monkeypatch):
    monkeypatch.setenv("FIFTY_MODEL", "does_not_exist")
    with pytest.raises(ValueError, match="unknown model"):
        _ = config.model

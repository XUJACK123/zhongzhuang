"""Passive gas spring: Fudan data transfer and physics interface.

The anchors are *transferred* from the Fudan model, so the first test recomputes
the transfer from Fudan's own XML and compares it with `configs/gas_spring.yaml`.
Nothing here claims the spring is calibrated on this robot.
"""

from __future__ import annotations

import numpy as np
import pytest

from fifty_mujoco.control.gas_spring import check_stroke, load_springs, spring_force
from fifty_mujoco.control.spawn import load_spawn_spec, resolve_spawn_height
from fifty_mujoco.control.spring_audit import audit_springs
from fifty_mujoco.envs.cpu_env import FiftyCpuEnv

FUDAN_XML = (
    "ref_document/fudan_rl_wheel_leg-main/mujoco/assert_now/infantry_binglian_yuntai/"
    "infantry_V2/meshes/mjmodel.xml"
)

# Fudan site name -> (Fudan body, our body, our joint whose origin is the reference)
FUDAN_SPRING_MAP = {
    "gas_right": {
        "a": ("right_lf0_qitanhuang", "rf0_Link", "r0_link", "rf0_Joint"),
        "b": ("right_lf1_qitanhuang", "rf1_Link", "r1_link", "rf1_Joint"),
    },
    "gas_left": {
        "a": ("left_lf0_qitanhuang", "lf0_Link", "l0_link", "lf0_Joint"),
        "b": ("left_lf1_qitanhuang", "lf1_Link", "l1_link", "lf1_Joint"),
    },
}


@pytest.fixture(scope="module")
def env(built_assets, config):
    _, paths = built_assets
    return FiftyCpuEnv(paths.train, config)


def test_two_springs_one_per_side(config):
    springs = load_springs(config)
    assert len(springs) == 2
    assert {spring.side for spring in springs} == {"right", "left"}
    for spring in springs:
        assert spring.force_ref_n == 300.0
        assert spring.parameter_status == "reference_300n"
        assert spring.body_a != spring.body_b


def test_anchors_match_the_fudan_model(config, env):
    """The anchor offsets from the hip / knee joint must equal Fudan's."""
    import mujoco

    fudan_path = config.workspace_path(FUDAN_XML)
    fudan = mujoco.MjModel.from_xml_path(str(fudan_path))
    fudan_data = mujoco.MjData(fudan)
    mujoco.mj_forward(fudan, fudan_data)

    spec = load_spawn_spec(config)
    resolve_spawn_height(env, spec)

    springs = {spring.name: spring for spring in load_springs(config)}
    for spring_name, mapping in FUDAN_SPRING_MAP.items():
        spring = springs[spring_name]
        for side, (site_name, _fudan_body, our_body, joint_name) in mapping.items():
            site_id = mujoco.mj_name2id(fudan, mujoco.mjtObj.mjOBJ_SITE, site_name)
            assert site_id >= 0, site_name
            fudan_site = fudan_data.site_xpos[site_id].copy()
            fudan_joint = mujoco.mj_name2id(fudan, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            fudan_offset = fudan_site - fudan_data.xanchor[fudan_joint]

            our_site = env.data.site_xpos[
                mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, f"{spring.name}_{side}")
            ]
            our_joint = env.joint_ids[joint_name]
            our_offset = our_site - env.data.xanchor[our_joint]

            assert np.allclose(our_offset, fudan_offset, atol=2e-6), (
                f"{spring.name}.{side}: offset {our_offset} != Fudan {fudan_offset}"
            )


def test_spring_geometry_at_spawn(config, env):
    spec = load_spawn_spec(config)
    resolve_spawn_height(env, spec)
    state = env.spring_state()
    assert set(state) == {"gas_right", "gas_left"}
    for name, entry in state.items():
        # Sanity: a strut between the hip and the knee, not a degenerate point.
        assert 0.15 < entry["length_m"] < 0.35, name
        assert entry["force_n"] == 300.0
        assert abs(entry["length_m"] - entry["tendon_length_m"]) <= 1e-9


def test_force_model_constant_and_clipped():
    from fifty_mujoco.control.gas_spring import SpringParams

    params = SpringParams(
        name="test", side="right", body_a="a", xyz_a=np.zeros(3), body_b="b", xyz_b=np.zeros(3),
        length_min_m=None, length_max_m=None, length_ref_m=None,
        force_ref_n=300.0, stiffness_n_per_m=0.0, damping_n_s_per_m=0.0, friction_n=0.0,
        velocity_smoothing_m_s=None, force_min_n=0.0, force_max_n=300.0,
        parameter_status="test", stroke_status="unverified", source=None,
    )
    assert spring_force(params, 0.2, 0.0) == 300.0
    assert spring_force(params, 0.05, 5.0) == 300.0  # clipped to force_max


def test_stroke_is_reported_as_unverified(config):
    springs = load_springs(config)
    for spring in springs:
        report = check_stroke(spring, 0.23)
        assert report["status"] == "unverified"
        assert spring.stroke_status == "unverified"


def test_no_net_external_force(env, config):
    """An internal spring pair must not accelerate the whole robot."""
    spec = load_spawn_spec(config)
    resolve_spawn_height(env, spec)
    env.zero_joint_motors()
    env.apply_passive_springs()
    env.forward()
    base = np.asarray(env.data.qfrc_actuator)[:6]
    assert np.max(np.abs(base)) <= 1e-9


def test_passive_force_survives_motors_off(env, config):
    spec = load_spawn_spec(config)
    resolve_spawn_height(env, spec)
    env.zero_joint_motors()
    env.apply_passive_springs()
    env.forward()
    joint_dofs = [env.dof_adr[name] for name in config.joint_order]
    assert np.max(np.abs(np.asarray(env.data.qfrc_actuator)[joint_dofs])) > 1.0


def test_spring_audit_interface_passes_but_gate_stays_open(config, env):
    report = audit_springs(config, env)
    for name, check in report["checks"].items():
        assert check["passed"], f"{name} failed: {check}"
    assert report["status"] == "passed"
    # Gate: anchors are transferred, stroke unknown -> P3 is not closed.
    assert report["gate_closed"] is False
    assert report["calibrated"] is False
    assert any("stroke" in reason for reason in report["gate"]["reasons"])

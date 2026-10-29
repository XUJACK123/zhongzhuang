"""LQR teacher contract (doc section 11.11).

The teacher is not validated here - nothing in a unit test can prove that a
controller stands.  What *is* pinned:

* the reduced model is built from the MJCF, not from inherited constants,
* the wheel-pendulum signs are the ones the derivation implies,
* the Riccati solve returns a stabilising gain,
* the torque -> action adapter is the exact inverse of the mixed PD and flags
  saturation instead of hiding it,
* the teacher exposes ``bind``/``reset`` and returns a finite six-channel action.
"""

from __future__ import annotations

import numpy as np
import pytest

from fifty_mujoco.contracts import names
from fifty_mujoco.control.lqr_teacher import (
    LqrTeacher,
    WheelLegParameters,
    solve_discrete_lqr,
    wheel_pendulum_matrices,
)
from fifty_mujoco.control.teacher_adapter import (
    action_to_torques,
    default_gains,
    spring_feedforward_torque,
    torques_to_action,
)


@pytest.fixture(scope="module")
def env(config, built_assets):
    del built_assets
    from fifty_mujoco.envs.fifty_env import EnvOptions, FiftyVecEnv

    return FiftyVecEnv(
        config,
        EnvOptions(
            num_envs=1,
            seed=21,
            episode_length_s=5.0,
            fixed_command=(0.0, 0.0, 0.464751),
        ),
    )


def test_riccati_iteration_stabilises_a_double_integrator():
    dt = 0.01
    a = np.array([[1.0, dt], [0.0, 1.0]])
    b = np.array([[0.5 * dt * dt], [dt]])
    q = np.eye(2)
    r = np.array([[1.0]])
    k = solve_discrete_lqr(a, b, q, r)
    assert np.all(np.isfinite(k))
    closed = a - b @ k
    assert max(abs(np.linalg.eigvals(closed))) < 1.0


def test_wheel_pendulum_signs():
    params = WheelLegParameters(
        total_mass_kg=20.0,
        wheel_radius_m=0.06,
        wheel_inertia_kg_m2=0.01,
        com_above_axle_m=0.3,
        leg_length_m=0.4,
    )
    a, b = wheel_pendulum_matrices(params)
    # positive wheel torque accelerates forward and pitches the body backwards
    assert b[0, 0] > 0.0
    assert b[2, 0] < 0.0
    # gravity makes the pitch unstable (positive pitch feeds positive pitch rate)
    assert a[2, 1] > 0.0


def test_teacher_measures_its_parameters_from_the_model(config, env):
    teacher = LqrTeacher(config).bind(env)
    params = teacher.params
    assert params.total_mass_kg == pytest.approx(23.2596305, rel=1e-6)
    assert params.wheel_radius_m == pytest.approx(0.06, abs=1e-6)
    assert 0.2 < params.com_above_axle_m < 0.5
    assert 0.35 < params.leg_length_m < 0.5
    assert teacher.k_wheel is not None and np.all(np.isfinite(teacher.k_wheel))
    assert set(teacher.spawn_leg_angle) == {"r", "l"}


def test_teacher_action_is_finite_and_six_dimensional(config, env):
    teacher = LqrTeacher(config).bind(env)
    action = teacher(env.obs, None)
    assert action.shape == (1, names.NUM_ACTIONS)
    assert np.all(np.isfinite(action))
    teacher.reset()
    assert np.allclose(teacher.last_action, 0.0)
    assert not teacher.saturation.any()


def test_teacher_leg_geometry_matches_the_env_theta(config, env):
    """The teacher's leg angle must be the same quantity the reward uses."""
    teacher = LqrTeacher(config).bind(env)
    theta = env._theta0(0)
    for side in ("r", "l"):
        geometry = teacher.leg_geometry(env, 0, side)
        assert geometry.angle_rad == pytest.approx(theta[side], abs=1e-9)
        assert geometry.jacobian.shape == (2, 2)


def test_torque_action_round_trip_is_exact_below_the_limit(config):
    gains = default_gains(config)
    rng = np.random.default_rng(3)
    position = np.array([config.default_angles[name] for name in names.MODEL_JOINT_ORDER])
    velocity = rng.normal(0.0, 0.2, 6)
    for _ in range(20):
        torque = rng.uniform(-0.8, 0.8, 6) * gains.limits()
        result = torques_to_action(torque, position, velocity, config, gains)
        assert not result.saturated.any()
        applied = action_to_torques(result.action, position, velocity, config, gains)
        assert np.allclose(applied, torque, atol=1e-9)


def test_saturation_is_flagged_not_hidden(config):
    gains = default_gains(config)
    position = np.array([config.default_angles[name] for name in names.MODEL_JOINT_ORDER])
    velocity = np.zeros(6)
    torque = gains.limits() * 3.0
    result = torques_to_action(torque, position, velocity, config, gains)
    assert result.saturated.all()
    assert not result.valid
    assert np.allclose(np.abs(result.applied_torque), gains.limits())


def test_spring_feedforward_reports_the_passive_torque(config, env):
    if not config.gas_spring_configured:
        pytest.skip("no gas spring on this model")
    torque = spring_feedforward_torque(config, env, 0)
    assert torque.shape == (6,)
    assert np.all(np.isfinite(torque))
    assert np.abs(torque).max() > 0.0

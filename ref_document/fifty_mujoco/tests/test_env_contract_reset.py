"""Reset / history / terminal / command contract (doc section 5.1 - 5.3).

These are the regressions the previous training runs could not see:

* the observation returned after an automatic reset was the *terminal* one,
* the terminal history did not contain the terminal frame, so a timeout
  bootstrapped from a latent that described the previous step,
* ``dones`` stayed ``True`` for a whole step after a reset,
* the GPU ``reset`` wrote into an advanced-index temporary and never zeroed
  the motor commands,
* a command change was not visible until the next physics step,
* the CPU and GPU spring features used different fields and scaling.

Everything here runs on the CPU model, so it executes on a machine without a
GPU; the GPU parity items that need CUDA live in ``test_warp_env.py`` and are
reported as skipped, never as passed.
"""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from fifty_mujoco.contracts import names
from fifty_mujoco.envs.fifty_env import EnvOptions, FiftyVecEnv
from fifty_mujoco.mdp import observations as obs_lib
from fifty_mujoco.mdp.commands import CommandRanges


@pytest.fixture(scope="module")
def standing_height(config, built_assets):
    del built_assets  # the session fixture guarantees the MJCF exists
    from fifty_mujoco.control.spawn import load_spawn_spec, resolve_spawn_height
    from fifty_mujoco.envs.cpu_env import FiftyCpuEnv
    from fifty_mujoco.paths import assets_dir

    env = FiftyCpuEnv(assets_dir() / "mjcf" / "fifty_train.xml", config)
    root_z = resolve_spawn_height(env, load_spawn_spec(config))[0]
    return root_z - float(config.spawn["spawn"]["wheel_clearance_m"])


def _env(config, height, num_envs=1, **kwargs):
    return FiftyVecEnv(
        config,
        EnvOptions(
            num_envs=num_envs,
            seed=3,
            fixed_command=(0.0, 0.0, height),
            command_ranges=CommandRanges(resample_time_s=1e9),
            **kwargs,
        ),
    )


def _roll_over(env: FiftyVecEnv, index: int = 0) -> None:
    """Put the robot on its side so the tilt termination fires deterministically."""
    data = env.data[index]
    angle = np.radians(80.0)
    data.qpos[3:7] = np.array([np.cos(angle / 2.0), np.sin(angle / 2.0), 0.0, 0.0])
    mujoco.mj_forward(env.model, data)


def test_terminal_history_ends_with_the_terminal_observation(config, standing_height):
    """A timeout must bootstrap from the state it actually timed out in."""
    env = _env(config, standing_height)
    obs, _history = env.get_observations()
    _roll_over(env)
    for _ in range(40):
        obs_before = obs[0].copy()
        obs, _priv, _rew, done, infos, _hist = env.step(np.zeros((1, 6)))
        if done[0]:
            terminal = infos["terminal_history"][0].reshape(
                names.HISTORY_LENGTH, names.ACTOR_OBS_DIM
            )
            terminal_obs = infos["terminal_obs"][0]
            assert np.allclose(terminal[-1], terminal_obs[3 : 3 + names.ACTOR_OBS_DIM])
            assert np.allclose(terminal[-2], obs_before)
            return
    pytest.fail("the forced tilt failure never happened")


def test_observation_after_reset_describes_the_spawn_state(config, standing_height):
    env = _env(config, standing_height)
    _roll_over(env)
    for _ in range(40):
        obs, privileged, _rew, done, _infos, history = env.step(np.zeros((1, 6)))
        if done[0]:
            assert np.allclose(obs[0, 3:6], [0.0, 0.0, -1.0], atol=1e-3)
            assert np.allclose(obs[0, 19:25], 0.0)
            assert np.allclose(obs[0, 0:3], 0.0, atol=1e-6)
            frames = history.reshape(1, names.HISTORY_LENGTH, names.ACTOR_OBS_DIM)[0]
            for frame in frames:
                assert np.allclose(frame, obs[0])
            assert env.data[0].qpos[2] == pytest.approx(env.spawn_z, abs=1e-6)
            assert np.allclose(privileged[0, 0:3], 0.0, atol=1e-6)
            return
    pytest.fail("the forced tilt failure never happened")


def test_dones_is_reported_for_the_transition_that_ended(config, standing_height):
    """``step`` returns the done flag of its own transition, not a stale one."""
    env = _env(config, standing_height)
    _roll_over(env)
    ended = False
    for _ in range(40):
        _, _, _, done, _infos, _ = env.step(np.zeros((1, 6)))
        if done[0]:
            ended = True
            continue
        if ended:
            assert not env.dones.any()
            assert not env.time_outs.any()
            return
    pytest.fail("the forced tilt failure never happened")


def test_explicit_reset_clears_dones_and_zeroes_control(config, standing_height):
    env = _env(config, standing_height)
    env.step(np.ones((1, 6)))
    _roll_over(env)
    for _ in range(40):
        env.step(np.zeros((1, 6)))
        if env.dones.any():
            break
    env.reset()
    assert not env.dones.any()
    assert not env.time_outs.any()
    assert np.allclose(env.data[0].ctrl, 0.0)
    assert np.allclose(env.actions, 0.0)
    assert np.allclose(env.motor_torques, 0.0)
    assert np.allclose(env.joint_acceleration, 0.0)


def test_set_commands_is_visible_immediately(config, standing_height):
    env = _env(config, standing_height, num_envs=2)
    obs_before, history_before = env.get_observations()
    joint_acc_before = env.joint_acceleration.copy()
    velocity_before = env.previous_joint_velocity.copy()

    env.set_commands(np.array([[0.15, -0.2, standing_height], [0.0, 0.0, standing_height]]))

    obs, history = env.get_observations()
    assert np.allclose(obs[0, 6:9], np.array([0.15, -0.2, standing_height]) * obs_lib.COMMAND_SCALE)
    frames = history.reshape(2, names.HISTORY_LENGTH, names.ACTOR_OBS_DIM)
    assert np.allclose(frames[0, -1, 6:9], obs[0, 6:9])
    # the refresh must not advance history, physics or the acceleration estimate
    assert np.allclose(frames[0, :-1], history_before.reshape(2, 5, 25)[0, :-1])
    assert np.allclose(env.joint_acceleration, joint_acc_before)
    assert np.allclose(env.previous_joint_velocity, velocity_before)
    assert np.allclose(obs[1], obs_before[1])
    # the privileged copy of obs carries the same command field
    privileged = env.get_privileged_observations()
    assert np.allclose(privileged[0, 9:12], obs[0, 6:9])


def test_commands_resample_on_the_episode_clock(config, standing_height):
    """Not every env flips its command at the same PPO update boundary."""
    del standing_height
    ranges = CommandRanges(resample_time_s=0.5)
    env = FiftyVecEnv(config, EnvOptions(num_envs=2, seed=4, command_ranges=ranges))
    first = env.commands.values.copy()
    env._resample_expired_commands(0)  # 10 ms elapsed, nothing due yet
    assert np.allclose(env.commands.values[0], first[0])
    for _ in range(60):
        env._resample_expired_commands(0)
    assert not np.allclose(env.commands.values[0], first[0])
    assert np.allclose(env.commands.values[1], first[1])
    assert 0.0 < env.commands.resample_timer[0] <= ranges.resample_time_s


def test_fixed_command_never_resamples(config, standing_height):
    env = _env(config, standing_height)
    before = env.commands.values.copy()
    for _ in range(200):
        env._resample_expired_commands(0)
    assert np.allclose(env.commands.values, before)


def test_spring_features_keep_reference_and_actual_force_apart(config, standing_height):
    """The contract is length*5, rate, reference/300, actual/300, per spring."""
    if not config.gas_spring_configured:
        pytest.skip("gas spring is not configured on this model")
    from fifty_mujoco.control.gas_spring import reference_force

    env = _env(config, standing_height)
    for _ in range(10):
        env.step(np.array([[0.3, 0.0, 0.0, 0.3, 0.0, 0.0]]))

    lengths, rates, forces, _ = env._spring_state(0)
    count = env.spring_count
    expected = np.concatenate(
        [
            lengths * 5.0,
            rates,
            np.array([reference_force(spring) for spring in env.springs]) / 300.0,
            forces / 300.0,
        ]
    )
    spring_part = env.get_privileged_observations()[0, names.BASE_PRIVILEGED_DIM :]
    assert spring_part.size == names.GAS_SPRING_FEATURES_PER_SPRING * count
    assert np.allclose(spring_part, expected, atol=1e-6)
    # A pure constant-force model makes the two channels coincide by definition;
    # as soon as stiffness/damping/friction exist they must not be conflated.
    if any(spring.stiffness_n_per_m or spring.damping_n_s_per_m or spring.friction_n for spring in env.springs):
        reference_channels = spring_part[2 * count : 3 * count]
        actual_channels = spring_part[3 * count : 4 * count]
        assert not np.allclose(reference_channels, actual_channels)


def test_privileged_dimension_matches_contract(config, standing_height):
    env = _env(config, standing_height)
    assert env.num_privileged_obs == names.BASE_PRIVILEGED_DIM + 4 * env.spring_count
    assert env.critic_input_dim == env.num_privileged_obs + names.ENCODER_LATENT_DIM


def test_timeout_is_separate_from_failure(config, standing_height):
    """A short episode limit produces a timeout with a terminal snapshot."""
    env = _env(config, standing_height, episode_length_s=0.05)
    for _ in range(10):
        _, _, _, done, infos, _hist = env.step(np.zeros((1, 6)))
        if done[0]:
            assert infos["time_outs"][0]
            assert infos["termination_reason"][0] == "time_out"
            assert infos["terminal_history"].shape == (1, names.HISTORY_DIM)
            assert infos["terminal_obs"].shape == (1, env.num_privileged_obs)
            return
    pytest.fail("the timeout never fired")


def test_step_stats_describe_the_pre_reset_state(config, standing_height):
    """The evaluator must never score the spawn pose after a fall."""
    env = _env(config, standing_height)
    _roll_over(env)
    for _ in range(40):
        _, _, _, done, infos, _ = env.step(np.zeros((1, 6)))
        if done[0]:
            stats = infos["step_stats"]
            assert stats["tilt_deg"][0] > 30.0
            assert stats["height"][0] < env.spawn_z
            assert stats["alive"][0]
            return
    pytest.fail("the forced tilt failure never happened")

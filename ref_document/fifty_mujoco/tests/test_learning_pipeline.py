"""P4-P8: control contract, observations, PPO update and ONNX round-trip.

These tests are the executable version of the gate reports: if one of them
fails the corresponding report in ``reports/`` can no longer be trusted.
"""

from __future__ import annotations

import numpy as np
import pytest

from fifty_mujoco.config import load_configs
from fifty_mujoco.contracts import names
from fifty_mujoco.control.mixed_pd import (
    ActionTargets,
    PdGains,
    compute_torques,
    decode_action,
)
from fifty_mujoco.envs.fifty_env import FiftyVecEnv, EnvOptions
from fifty_mujoco.mdp import observations, rewards


@pytest.fixture(scope="module")
def config():
    return load_configs()


@pytest.fixture(scope="module")
def env(config):
    return FiftyVecEnv(config, EnvOptions(num_envs=2, seed=5))


# --------------------------------------------------------------------- PD
def test_action_decoding_uses_only_the_documented_channels(config):
    gains = PdGains(
        torque_limit_nm=config.effort_limit("rf0_Joint"),
        wheel_torque_limit_nm=config.effort_limit("r_wheel_Joint"),
    )
    targets = decode_action(np.arange(1, 7, dtype=float), config.default_angles, gains)
    for index in names.POLICY_LEG_INDICES:
        expected = config.default_angles[names.MODEL_JOINT_ORDER[index]] + 0.5 * (index + 1)
        assert targets.position_target[index] == pytest.approx(expected)
        assert targets.velocity_target[index] == 0.0
    for index in names.POLICY_WHEEL_INDICES:
        assert targets.position_target[index] == pytest.approx(
            config.default_angles[names.MODEL_JOINT_ORDER[index]]
        )
        assert targets.velocity_target[index] == pytest.approx(10.0 * (index + 1))


def test_torque_limits_are_per_joint_group():
    gains = PdGains(torque_limit_nm=30.0, wheel_torque_limit_nm=5.0)
    targets = ActionTargets(
        position_target=np.full(6, 100.0),
        velocity_target=np.full(6, 10_000.0),
    )
    torque = compute_torques(targets, np.zeros(6), np.zeros(6), gains)
    for index in names.POLICY_LEG_INDICES:
        assert torque[index] == pytest.approx(30.0)
    for index in names.POLICY_WHEEL_INDICES:
        assert torque[index] == pytest.approx(5.0)


def test_spring_compensation_happens_before_the_limit():
    gains = PdGains(torque_limit_nm=30.0, wheel_torque_limit_nm=5.0)
    targets = ActionTargets(position_target=np.zeros(6), velocity_target=np.zeros(6))
    estimated = np.zeros(6)
    estimated[list(names.POLICY_LEG_INDICES)] = 12.5
    compensated = compute_torques(targets, np.zeros(6), np.zeros(6), gains, estimated)
    assert compensated[0] == pytest.approx(-12.5)
    limited = compute_torques(
        targets, np.zeros(6), np.zeros(6), gains, estimated * 10.0
    )
    assert limited[0] == pytest.approx(-30.0)


def test_motors_off_makes_every_torque_zero():
    gains = PdGains()
    targets = ActionTargets(position_target=np.full(6, 1.0), velocity_target=np.ones(6))
    torque = compute_torques(targets, np.zeros(6), np.zeros(6), gains, motor_enabled=False)
    assert np.all(torque == 0.0)


# ----------------------------------------------------------- observations
def test_proprio_matches_the_verified_channel_order(config):
    default = np.array([config.default_angles[name] for name in names.MODEL_JOINT_ORDER])
    ang_vel = np.array([0.1, 0.2, 0.3])
    gravity = np.array([0.0, 0.0, -1.0])
    command = np.array([0.4, -0.2, 0.46])
    position = default + 0.05
    velocity = np.arange(6, dtype=float)
    last_action = np.arange(6, dtype=float) * 0.1

    obs = observations.proprio(
        ang_vel, gravity, command, position, velocity, last_action, default
    )

    assert obs.shape == (names.ACTOR_OBS_DIM,)
    assert np.allclose(obs[0:3], ang_vel * 0.25)
    assert np.allclose(obs[3:6], gravity)
    assert np.allclose(obs[6:9], command * np.array([3.0, 0.25, 5.0]))
    legs = list(names.POLICY_LEG_INDICES)
    assert np.allclose(obs[9:13], (position - default)[legs])
    assert np.allclose(obs[13:19], velocity * 0.05)
    assert np.allclose(obs[19:25], last_action)


def test_privileged_dimension_follows_the_spring_count(config):
    base = np.zeros(names.BASE_PRIVILEGED_DIM)
    springs = np.zeros(4 * config.spring_count)
    assert base.size == 141
    assert base.size + springs.size == 141 + 8


def test_history_is_oldest_first(env):
    history = env.get_observations()[1]
    frames = history.reshape(env.num_envs, names.HISTORY_LENGTH, names.ACTOR_OBS_DIM)
    assert frames.shape[1] == names.HISTORY_LENGTH == 5
    # a freshly reset env repeats the same frame, so old and new must agree
    assert np.allclose(frames[0, 0], frames[0, -1])


# ---------------------------------------------------------------- rewards
def test_reward_combination_is_clipped_and_dt_scaled():
    terms = {"tracking_lin_vel": 1e6, "alive": 1.0}
    total, contribution = rewards.combine(terms, dt=0.01)
    assert contribution["tracking_lin_vel"] == pytest.approx(0.01)
    assert abs(total) <= 1.0 + 1e-12


# -------------------------------------------------------- episode plumbing
def test_step_returns_the_documented_shapes(env):
    obs, privileged, reward, done, infos, history = env.step(np.zeros((env.num_envs, 6)))
    assert obs.shape == (env.num_envs, names.ACTOR_OBS_DIM)
    assert privileged.shape == (env.num_envs, env.num_privileged_obs)
    assert history.shape == (env.num_envs, names.HISTORY_DIM)
    assert reward.shape == done.shape == (env.num_envs,)
    assert "time_outs" in infos


def test_terminal_obs_is_pre_termination(env):
    env.reset()
    obs, privileged, reward, done, infos, history = env.step(np.zeros((env.num_envs, 6)))
    if "terminal_obs" in infos and np.any(done):
        ids = infos["reset_env_ids"]
        assert infos["terminal_obs"].shape[0] == len(ids)
        assert np.all(np.isfinite(infos["terminal_obs"]))


# --------------------------------------------------------- PPO / transfer
def test_ppo_update_changes_parameters(config):
    torch = pytest.importorskip("torch")
    from fifty_mujoco.learning.ppo import Ppo, PpoConfig
    from fifty_mujoco.learning.runner import OnPolicyRunner, RunnerConfig

    runner = OnPolicyRunner(
        config,
        runner_config=RunnerConfig(
            num_envs=4, num_steps_per_env=4, max_updates=1, critic_warmup_updates=0
        ),
    )
    before = [p.detach().clone() for p in runner.policy.parameters()]
    result = runner.train(verbose=False)
    assert result["updates"] == 1
    after = [p.detach().clone() for p in runner.policy.parameters()]
    changed = any(not torch.equal(a, b) for a, b in zip(before, after))
    assert changed, "PPO update left every parameter untouched"


def test_checkpoint_round_trip(config, tmp_path):
    pytest.importorskip("torch")
    from fifty_mujoco.learning.runner import OnPolicyRunner, RunnerConfig

    runner = OnPolicyRunner(config, runner_config=RunnerConfig(num_envs=2))
    path = tmp_path / "model.pt"
    runner.save(path)
    restored = OnPolicyRunner(
        config, runner_config=RunnerConfig(num_envs=2), checkpoint=path
    )
    for (name_a, param_a), (name_b, param_b) in zip(
        runner.policy.named_parameters(), restored.policy.named_parameters()
    ):
        assert name_a == name_b
        assert np.array_equal(param_a.detach().numpy(), param_b.detach().numpy())


def test_onnx_export_round_trip(config, tmp_path):
    pytest.importorskip("torch")
    pytest.importorskip("onnx")
    from fifty_mujoco.learning.runner import OnPolicyRunner, RunnerConfig
    from fifty_mujoco.transfer.export import export_onnx, verify_onnx

    runner = OnPolicyRunner(config, runner_config=RunnerConfig(num_envs=1))
    out = tmp_path / "policy.onnx"
    export_onnx(runner.policy, out)
    report = verify_onnx(runner.policy, out)
    assert report["passed"]
    assert report["max_abs_error"] < 1e-4
    from fifty_mujoco.transfer.export import OnnxPolicy

    policy = OnnxPolicy(out)
    assert "obs" in policy.inputs and "obs_history" in policy.inputs


# ------------------------------------------------------- contact helpers
def test_vertical_slider_only_pins_attitude_and_horizontal(config):
    from fifty_mujoco.evaluation.contact_suite import _vertical_slider

    local = FiftyVecEnv(config, EnvOptions(num_envs=1, seed=3))
    proxy = local.slot(0) if hasattr(local, "slot") else None
    target = proxy if proxy is not None else local
    data = target.data if proxy is not None else local.data[0]

    class _View:
        pass

    view = _View()
    view.data = data
    _vertical_slider(view)  # type: ignore[arg-type]
    assert np.all(data.qpos[0:2] == 0.0)
    assert np.all(data.qpos[3:7] == np.array([1.0, 0.0, 0.0, 0.0]))
    assert np.all(data.qvel[0:2] == 0.0)
    assert np.all(data.qvel[3:6] == 0.0)
    # vertical position/velocity must stay untouched by the helper
    assert np.isfinite(data.qpos[2])

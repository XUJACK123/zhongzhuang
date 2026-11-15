import importlib.util
from pathlib import Path

import numpy as np
import pytest
import torch

from fifty_mujoco.envs.sample_env31 import SampleEnv31, rpy_from_wxyz, wilson_interval
from fifty_mujoco.learning.sample_ppo import SamplePolicy31, SamplePPO, gae
from fifty_mujoco.transfer.sample_checkpoint import load_sample_actor
from fifty_mujoco.transfer.sample_source import verified_artifact


def test_gae_bootstraps_terminal_timeouts_and_never_crosses_reset():
    # t0 timeout: r=1 + gamma*terminalV(10); t1 real failure: r=2,
    # even though V(next observation) would be 100. No carry crosses t0.
    rewards = torch.tensor([[1.], [2.]])
    values = torch.zeros_like(rewards)
    next_values = torch.tensor([[10.], [100.]])
    terminated = torch.tensor([[False], [True]])
    truncated = torch.tensor([[True], [False]])
    advantage, returns = gae(rewards, values, next_values, terminated, truncated, gamma=.9, lam=.95)
    torch.testing.assert_close(returns, torch.tensor([[10.], [2.]]))
    torch.testing.assert_close(advantage, returns)


def test_pretrained_actor_is_exact_and_critic_is_fresh_31_inputs():
    policy = SamplePolicy31(verified_artifact("model_3050.pt"))
    actor, _ = load_sample_actor(verified_artifact("model_3050.pt"))
    x = torch.randn(32, 31)
    torch.testing.assert_close(policy.actor(x), actor(x), atol=0, rtol=0)
    assert policy.critic[0].in_features == 31
    assert policy.origin["critic_restored"] is False


def test_critic_warmup_leaves_actor_and_exploration_unchanged_then_ppo_updates():
    torch.manual_seed(12)
    policy = SamplePolicy31(verified_artifact("model_3050.pt"))
    ppo = SamplePPO(policy)
    obs = torch.randn(2, 8, 31)*.1
    with torch.no_grad():
        dist = policy.distribution(obs.reshape(-1, 31))
        actions = dist.sample().reshape(2, 8, 6)
        logp = dist.log_prob(actions.reshape(-1, 6)).sum(-1).reshape(2, 8)
        values = policy.value(obs.reshape(-1, 31)).reshape(2, 8)
    rollout = dict(observation=obs, action=actions, log_prob=logp, value=values,
                   mean=dist.mean.reshape(2, 8, 6), std=dist.stddev.reshape(2, 8, 6),
                   reward=torch.randn(2, 8), next_value=torch.zeros(2, 8),
                   terminated=torch.zeros(2, 8, dtype=torch.bool), truncated=torch.zeros(2, 8, dtype=torch.bool))
    initial = {k: v.clone() for k, v in policy.state_dict().items()}
    ppo.update(rollout, warmup=True)
    for key in initial:
        if key.startswith("actor.") or key == "log_std":
            torch.testing.assert_close(policy.state_dict()[key], initial[key], atol=0, rtol=0)
    assert any(not torch.equal(v, initial[k]) for k, v in policy.state_dict().items() if k.startswith("critic."))
    result = ppo.update(rollout, warmup=False)
    assert result["final_full_rollout_kl"] <= .02
    assert any(not torch.equal(v, initial["actor."+k]) for k, v in policy.actor.state_dict().items())
    # No artificial increase to 0.01 on source channels whose std is smaller.
    assert policy.log_std.exp().min() < .01


def test_excessive_kl_proposal_restores_actor_and_adam_state():
    torch.manual_seed(51)
    policy = SamplePolicy31(verified_artifact("model_3050.pt"))
    ppo = SamplePPO(policy, actor_lr=.1)
    obs = torch.randn(1, 16, 31)*.1
    with torch.no_grad():
        distribution = policy.distribution(obs[0])
        actions = distribution.sample()
        values = policy.value(obs[0])
    rollout = dict(observation=obs, action=actions[None], log_prob=distribution.log_prob(actions).sum(-1)[None],
        value=values[None], reward=torch.randn(1, 16), next_value=torch.zeros(1, 16),
        terminated=torch.zeros(1, 16, dtype=torch.bool), truncated=torch.zeros(1, 16, dtype=torch.bool),
        mean=distribution.mean[None], std=distribution.stddev[None])
    initial = {k: v.clone() for k, v in policy.actor.state_dict().items()}
    std_before = policy.log_std.clone()
    stats = ppo.update(rollout, epochs=1, batch_size=16)
    assert stats["actor_kl_rollbacks"] == 1
    assert stats["final_full_rollout_kl"] == 0
    assert not ppo.actor_optimizer.state
    for name, value in policy.actor.state_dict().items():
        torch.testing.assert_close(value, initial[name], atol=0, rtol=0)
    torch.testing.assert_close(policy.log_std, std_before, atol=0, rtol=0)


def test_env_reconstructed_timing_matches_previous_diagnostic_trajectory():
    tool = Path(__file__).resolve().parents[1]/"tools/sample_rollout_diagnostic.py"
    spec = importlib.util.spec_from_file_location("sample_old_diagnostic", tool)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actor, _ = load_sample_actor(verified_artifact("model_3050.pt"))
    env = SampleEnv31(episode_seconds=.1)
    obs = env.reset(10300, [.15, 0, .1])
    states = []
    for _ in range(10):
        with torch.no_grad():
            action = actor(torch.tensor(obs)[None])[0].numpy()
        obs, _, terminated, truncated, _ = env.step(action)
        assert not terminated
        states.append(env.data.qpos.copy())
    _, trace = module.trial(env.source, actor, [.15, 0, .1], 10300, .1)
    np.testing.assert_allclose(states, trace[:, 1:14], atol=1e-12, rtol=0)
    assert truncated
    env.finished = True
    with pytest.raises(RuntimeError):
        env.step([0]*6)


def test_heading_wrap_and_wilson_interval():
    np.testing.assert_allclose(rpy_from_wxyz([1, 0, 0, 0]), [0, 0, 0])
    lo, hi = wilson_interval(5, 5)
    assert lo == pytest.approx(.5655175, abs=1e-6)
    assert hi == pytest.approx(1)
    assert wilson_interval(0, 0) == [None, None]


def test_timeout_retains_terminal_observation_until_explicit_reset():
    env = SampleEnv31(episode_seconds=.01)
    initial = env.reset(12)
    terminal, _, failure, timeout, info = env.step([0]*6)
    assert timeout and not failure and info["failure"] is None
    assert env.data.time == pytest.approx(.01)
    saved = terminal.copy()
    reset = env.reset(12)
    np.testing.assert_array_equal(initial, reset)
    np.testing.assert_array_equal(terminal, saved)
    assert not np.array_equal(terminal, reset)


def test_rotating_root_offset_does_not_create_false_command_point_translation():
    import mujoco
    env = SampleEnv31()
    env.reset()
    point = env._point().copy()
    angle = .3
    env.data.qpos[3:7] = [np.cos(angle/2), 0, 0, np.sin(angle/2)]
    mujoco.mj_forward(env.model, env.data)
    displacement = env._point()-env.data.qpos[:3]
    env.data.qpos[:3] = point-displacement
    mujoco.mj_forward(env.model, env.data)
    np.testing.assert_allclose(env._point(), point, atol=1e-15)

"""Rollout storage for the sequence policy (doc sections 5.3 and 7.3)."""

from __future__ import annotations

import torch


class RolloutStorage:
    def __init__(
        self,
        num_steps: int,
        num_envs: int,
        num_obs: int,
        num_privileged: int,
        num_history: int,
        num_actions: int,
        device: str = "cpu",
    ):
        self.num_steps = int(num_steps)
        self.num_envs = int(num_envs)
        self.device = device
        self.observations = torch.zeros(num_steps, num_envs, num_obs, device=device)
        self.history = torch.zeros(num_steps, num_envs, num_history, device=device)
        self.privileged = torch.zeros(num_steps, num_envs, num_privileged, device=device)
        self.actions = torch.zeros(num_steps, num_envs, num_actions, device=device)
        self.actions_log_prob = torch.zeros(num_steps, num_envs, device=device)
        self.values = torch.zeros(num_steps, num_envs, device=device)
        self.rewards = torch.zeros(num_steps, num_envs, device=device)
        self.dones = torch.zeros(num_steps, num_envs, device=device)
        self.returns = torch.zeros(num_steps, num_envs, device=device)
        self.advantages = torch.zeros(num_steps, num_envs, device=device)
        self.mu = torch.zeros(num_steps, num_envs, num_actions, device=device)
        self.sigma = torch.zeros(num_steps, num_envs, num_actions, device=device)
        self.terminal_privileged = torch.zeros(num_steps, num_envs, num_privileged, device=device)
        self.timeouts = torch.zeros(num_steps, num_envs, device=device)
        self.step = 0

    def add(
        self,
        observations,
        history,
        privileged,
        actions,
        log_prob,
        values,
        rewards,
        dones,
        mu,
        sigma,
        terminal_privileged=None,
        timeouts=None,
    ) -> None:
        step = self.step
        self.observations[step] = observations
        self.history[step] = history
        self.privileged[step] = privileged
        self.actions[step] = actions
        self.actions_log_prob[step] = log_prob
        self.values[step] = values
        self.rewards[step] = rewards
        self.dones[step] = dones
        self.mu[step] = mu
        self.sigma[step] = sigma
        if terminal_privileged is not None:
            self.terminal_privileged[step] = terminal_privileged
        if timeouts is not None:
            self.timeouts[step] = timeouts
        self.step += 1

    def clear(self) -> None:
        self.step = 0

    @property
    def full(self) -> bool:
        return self.step >= self.num_steps

    def compute_returns(
        self,
        last_values: torch.Tensor,
        gamma: float,
        lam: float,
        terminal_values: torch.Tensor | None = None,
        normalize: bool = True,
    ) -> None:
        """GAE with explicit timeout bootstrap.

        A failure bootstraps with zero; a timeout bootstraps with
        ``V(terminal observation)`` supplied by the caller.
        """
        if terminal_values is None:
            terminal_values = torch.zeros_like(self.values)
        last_advantage = torch.zeros_like(last_values)
        for step in reversed(range(self.num_steps)):
            next_values = last_values if step == self.num_steps - 1 else self.values[step + 1]
            done = self.dones[step]
            timeout = self.timeouts[step]
            bootstrap_value = torch.where(done > 0.5, terminal_values[step], next_values)
            bootstrap_mask = torch.where(done > 0.5, timeout, torch.ones_like(done))
            delta = self.rewards[step] + gamma * bootstrap_value * bootstrap_mask - self.values[step]
            last_advantage = delta + gamma * lam * (1.0 - done) * last_advantage
            self.returns[step] = last_advantage + self.values[step]
        self.advantages = self.returns - self.values
        if normalize and self.advantages.numel() > 1:
            self.advantages = (self.advantages - self.advantages.mean()) / (self.advantages.std() + 1e-8)

    def mini_batches(self, num_mini_batches: int, generator: torch.Generator | None = None):
        batch = self.num_steps * self.num_envs
        permutation = torch.randperm(batch, generator=generator)
        size = batch // num_mini_batches
        obs = self.observations.reshape(batch, -1)
        hist = self.history.reshape(batch, -1)
        priv = self.privileged.reshape(batch, -1)
        actions = self.actions.reshape(batch, -1)
        logp = self.actions_log_prob.reshape(batch)
        values = self.values.reshape(batch)
        returns = self.returns.reshape(batch)
        adv = self.advantages.reshape(batch)
        mu = self.mu.reshape(batch, -1)
        sigma = self.sigma.reshape(batch, -1)
        for start in range(0, batch, size):
            idx = permutation[start : start + size]
            if idx.numel() == 0:
                continue
            yield (
                obs[idx], hist[idx], priv[idx], actions[idx], logp[idx], values[idx],
                returns[idx], adv[idx], mu[idx], sigma[idx], self.terminal_privileged.reshape(batch, -1)[idx],
            )

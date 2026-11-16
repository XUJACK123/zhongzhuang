"""Actor initialization from the sample .pt; a fresh 31-input critic and PPO.

No encoder or privileged critic is borrowed from the legacy 25/125 path. GAE
bootstraps time limits from terminal observations, never a reset observation.
"""

from __future__ import annotations

import copy

import torch
from torch import nn

from ..transfer.sample_checkpoint import load_sample_actor

TRAIN_SCHEMA = "sample_transfer_ppo31_v1"


class SamplePolicy31(nn.Module):
    def __init__(self, source_checkpoint, device="cpu"):
        super().__init__()
        restored, self.origin = load_sample_actor(source_checkpoint)
        self.actor = restored.actor.requires_grad_(True)
        self.critic = nn.Sequential(nn.Linear(31, 128), nn.ELU(), nn.Linear(128, 64),
                                    nn.ELU(), nn.Linear(64, 1))
        state = torch.load(source_checkpoint, weights_only=True, map_location="cpu")["model_state_dict"]
        self.log_std = nn.Parameter(state["std"].log().clone())
        self.to(device)

    def distribution(self, obs):
        return torch.distributions.Normal(self.actor(obs), self.log_std.exp().expand(obs.shape[0], -1))

    def value(self, obs):
        return self.critic(obs).squeeze(-1)


def gae(rewards, values, next_values, terminated, truncated, gamma=.99, lam=.95):
    """Explicit next_values are evaluated BEFORE caller resets any environment."""
    advantage = torch.zeros_like(rewards)
    carry = torch.zeros_like(rewards[0])
    for t in reversed(range(len(rewards))):
        bootstrap = (~terminated[t]).to(rewards.dtype)
        continuation = (~(terminated[t] | truncated[t])).to(rewards.dtype)
        delta = rewards[t] + gamma*next_values[t]*bootstrap - values[t]
        carry = delta + gamma*lam*continuation*carry
        advantage[t] = carry
    return advantage, advantage + values


class SamplePPO:
    def __init__(self, policy, actor_lr=1e-6, critic_lr=3e-4):
        self.policy = policy
        self.actor_optimizer = torch.optim.Adam([*policy.actor.parameters(), policy.log_std], lr=actor_lr)
        self.critic_optimizer = torch.optim.Adam(policy.critic.parameters(), lr=critic_lr)

    def update(self, rollout, warmup=False, epochs=2, batch_size=128):
        advantage, returns = gae(rollout["reward"], rollout["value"], rollout["next_value"],
                                 rollout["terminated"], rollout["truncated"])
        advantages = advantage.flatten()
        advantages = (advantages-advantages.mean())/(advantages.std(unbiased=False)+1e-8)
        observations = rollout["observation"].reshape(-1, 31)
        actions = rollout["action"].reshape(-1, 6)
        old_mean = rollout["mean"].reshape(-1, 6)
        old_std = rollout["std"].reshape(-1, 6)
        old_log_prob = rollout["log_prob"].flatten()
        target = returns.flatten()
        totals = dict(value_loss=0., policy_loss=0., entropy=0., kl=0.)
        batches = 0
        kl_guard_skips = 0
        kl_rollbacks = 0
        for _ in range(epochs):
            permutation = torch.randperm(len(observations), device=observations.device)
            for idx in permutation.split(batch_size):
                loss_v = (self.policy.value(observations[idx])-target[idx]).square().mean()
                self.critic_optimizer.zero_grad()
                loss_v.backward()
                nn.utils.clip_grad_norm_(self.policy.critic.parameters(), 1., error_if_nonfinite=True)
                self.critic_optimizer.step()
                distribution = self.policy.distribution(observations[idx])
                log_prob = distribution.log_prob(actions[idx]).sum(-1)
                ratio = (log_prob-old_log_prob[idx]).exp()
                loss_p = -torch.minimum(ratio*advantages[idx], ratio.clamp(.8, 1.2)*advantages[idx]).mean()
                entropy = distribution.entropy().sum(-1).mean()
                with torch.no_grad():
                    kl = torch.distributions.kl_divergence(
                        torch.distributions.Normal(old_mean[idx], old_std[idx]), distribution).sum(-1).mean()
                if not warmup and kl <= .02:
                    parameters = [*self.policy.actor.parameters(), self.policy.log_std]
                    before = [p.detach().clone() for p in parameters]
                    optimizer_before = copy.deepcopy(self.actor_optimizer.state_dict())
                    self.actor_optimizer.zero_grad()
                    (loss_p-.001*entropy).backward()
                    nn.utils.clip_grad_norm_([*self.policy.actor.parameters(), self.policy.log_std], 1., error_if_nonfinite=True)
                    self.actor_optimizer.step()
                    with torch.no_grad():
                        # Preserve the source's small channel-specific stds;
                        # a 0.01 floor would abruptly change two channels.
                        self.policy.log_std.clamp_(min=-6.9, max=0.)
                        # Verify the proposal over the complete rollout. Reject
                        # both parameter and Adam-state changes if it overshoots.
                        full_kl = torch.distributions.kl_divergence(
                            torch.distributions.Normal(old_mean, old_std),
                            self.policy.distribution(observations)).sum(-1).mean()
                        if not torch.isfinite(full_kl) or full_kl > .02:
                            for parameter, original in zip(parameters, before):
                                parameter.copy_(original)
                            self.actor_optimizer.load_state_dict(optimizer_before)
                            for group in self.actor_optimizer.param_groups:
                                group["lr"] *= .5
                            kl_rollbacks += 1
                elif not warmup:
                    kl_guard_skips += 1
                for name, value in zip(totals, (loss_v, loss_p, entropy, kl)):
                    totals[name] += float(value.detach())
                batches += 1
        if not all(torch.isfinite(v).all() for v in self.policy.state_dict().values()):
            raise FloatingPointError("nonfinite updated sample policy")
        with torch.no_grad():
            final_kl = torch.distributions.kl_divergence(torch.distributions.Normal(old_mean, old_std),
                self.policy.distribution(observations)).sum(-1).mean()
        return {**{name: value/batches for name, value in totals.items()},
                "critic_warmup": warmup, "actor_kl_guard_skips": kl_guard_skips,
                "actor_kl_rollbacks": kl_rollbacks, "final_full_rollout_kl": float(final_kl),
                "actor_learning_rate": self.actor_optimizer.param_groups[0]["lr"]}

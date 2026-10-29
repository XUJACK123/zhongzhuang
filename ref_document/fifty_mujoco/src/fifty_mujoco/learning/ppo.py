"""PPO with the Fudan gradient paths (doc section 7.3).

    actor   <- PPO loss + teacher action loss
    critic  <- value loss
    encoder <- simulation-truth velocity loss (separate optimiser)

The latent fed to the actor is detached, so the teacher loss can never train the
encoder (the original project behaves the same way).
"""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import nn

from .storage import RolloutStorage


@dataclass
class PpoConfig:
    learning_rate: float = 1e-4
    encoder_learning_rate: float = 1e-4
    max_learning_rate: float = 3e-4
    num_epochs: int = 5
    num_mini_batches: int = 4
    clip_param: float = 0.2
    value_loss_coef: float = 1.0
    entropy_coef: float = 0.01
    gamma: float = 0.99
    lam: float = 0.95
    desired_kl: float = 0.01
    max_grad_norm: float = 1.0
    encoder_loss_coef: float = 1.0
    encoder_max_grad_norm: float = 0.1
    teacher_coef: float = 0.1
    min_std: float = 1e-3
    #: Set by the runner's stage machine.  Zero means "critic warm-up": the
    #: actor, the exploration std and the encoder receive no gradient at all.
    policy_loss_scale: float = 1.0
    encoder_loss_scale: float = 1.0


class Ppo:
    def __init__(self, policy: nn.Module, config: PpoConfig):
        self.policy = policy
        self.config = config
        self.optimizer = torch.optim.Adam(
            [p for name, p in policy.named_parameters() if not name.startswith("encoder.")],
            lr=config.learning_rate,
        )
        # The encoder has its own optimiser so it can be saved and restored
        # independently (the original runner did not save it at all).
        self.encoder_optimizer = torch.optim.Adam(
            policy.encoder.parameters(), lr=config.encoder_learning_rate
        )
        self.teacher: nn.Module | None = None
        self.teacher_std: torch.Tensor | None = None
        self.teacher_coef: float = 0.0

    # ------------------------------------------------------------------ helpers
    def set_teacher(self, teacher: nn.Module | None, std: torch.Tensor | None, coef: float) -> None:
        self.teacher = teacher
        self.teacher_std = std
        self.teacher_coef = float(coef)
        if teacher is not None:
            teacher.eval()
            for parameter in teacher.parameters():
                parameter.requires_grad_(False)

    def _teacher_loss(self, obs: torch.Tensor, history: torch.Tensor, mu: torch.Tensor) -> torch.Tensor:
        if self.teacher is None or self.teacher_coef <= 0.0:
            return torch.zeros((), device=obs.device)
        with torch.no_grad():
            target, _ = self.teacher.act_inference(obs, history)
        sigma = (self.teacher_std if self.teacher_std is not None else torch.ones_like(target) * 0.1)
        sigma = sigma.clamp(min=0.1)
        normalized = (mu - target) / sigma
        return torch.mean(normalized**2)

    # ------------------------------------------------------------------- update
    def update(self, storage: RolloutStorage) -> dict[str, float]:
        policy = self.policy
        stats: dict[str, float] = {}
        kl_total = 0.0
        batches = 0
        policy_loss_total = 0.0
        value_loss_total = 0.0
        entropy_total = 0.0
        encoder_loss_total = 0.0
        teacher_loss_total = 0.0

        generator = torch.Generator(device=storage.device).manual_seed(1234)
        for _ in range(self.config.num_epochs):
            for (
                obs, history, privileged, actions, old_log_prob, old_values,
                returns, advantages, old_mu, old_sigma, terminal_privileged,
            ) in storage.mini_batches(self.config.num_mini_batches, generator=generator):
                del old_values
                mean, std, latent = policy.evaluate(obs, history)
                distribution = torch.distributions.Normal(mean, std)
                log_prob = distribution.log_prob(actions).sum(dim=-1)
                ratio = torch.exp(log_prob - old_log_prob)
                surrogate = -torch.min(
                    ratio * advantages,
                    torch.clamp(ratio, 1.0 - self.config.clip_param, 1.0 + self.config.clip_param) * advantages,
                ).mean()

                values = policy.value(privileged, latent.detach()).squeeze(-1)
                value_loss = (returns - values).pow(2).mean()
                entropy = distribution.entropy().sum(dim=-1).mean()
                teacher_loss = self._teacher_loss(obs, history, mean)

                loss = (
                    self.config.policy_loss_scale * surrogate
                    + self.config.value_loss_coef * value_loss
                    - self.config.entropy_coef * entropy
                    + self.teacher_coef * teacher_loss
                )

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(policy.parameters(), self.config.max_grad_norm)
                self.optimizer.step()

                # Encoder: simulation-truth velocity (and optionally more) only.
                with torch.no_grad():
                    target = privileged[:, : policy.dims.latent_dim].detach()
                latent_raw = policy.encode(history)
                encoder_loss = (
                    self.config.encoder_loss_scale
                    * self.config.encoder_loss_coef
                    * (latent_raw - target).pow(2).mean()
                )
                self.encoder_optimizer.zero_grad()
                encoder_loss.backward()
                nn.utils.clip_grad_norm_(policy.encoder.parameters(), self.config.encoder_max_grad_norm)
                self.encoder_optimizer.step()

                with torch.no_grad():
                    kl = torch.mean((old_log_prob - log_prob) ** 2).item()
                kl_total += kl
                batches += 1
                policy_loss_total += float(surrogate.item())
                value_loss_total += float(value_loss.item())
                entropy_total += float(entropy.item())
                encoder_loss_total += float(encoder_loss.item())
                teacher_loss_total += float(teacher_loss.item())

        batches = max(1, batches)
        stats.update(
            {
                "policy_loss": policy_loss_total / batches,
                "value_loss": value_loss_total / batches,
                "entropy": entropy_total / batches,
                "encoder_loss": encoder_loss_total / batches,
                "teacher_loss": teacher_loss_total / batches,
                "kl": kl_total / batches,
                "teacher_coef": self.teacher_coef,
            }
        )
        self._adapt_learning_rate(stats["kl"])
        return stats

    def _adapt_learning_rate(self, kl: float) -> None:
        lr = self.optimizer.param_groups[0]["lr"]
        if kl > 2.0 * self.config.desired_kl:
            lr = max(lr / 1.5, 1e-6)
        elif kl < 0.5 * self.config.desired_kl:
            lr = min(lr * 1.5, self.config.max_learning_rate)
        self.optimizer.param_groups[0]["lr"] = lr

    # --------------------------------------------------------------------- i/o
    def state_dict(self) -> dict:
        return {
            "policy": self.policy.state_dict(),
            "optimizer": self.optimizer.state_dict(),
            "encoder_optimizer": self.encoder_optimizer.state_dict(),
            "learning_rate": self.optimizer.param_groups[0]["lr"],
            "encoder_learning_rate": self.encoder_optimizer.param_groups[0]["lr"],
        }

    def load_state_dict(self, state: dict) -> None:
        self.policy.load_state_dict(state["policy"])
        if "optimizer" in state:
            self.optimizer.load_state_dict(state["optimizer"])
        if "encoder_optimizer" in state:
            self.encoder_optimizer.load_state_dict(state["encoder_optimizer"])

    def act(self, obs: torch.Tensor, history: torch.Tensor, privileged: torch.Tensor):
        with torch.no_grad():
            mean, std, latent = self.policy.evaluate(obs, history)
            distribution = torch.distributions.Normal(mean, std)
            actions = distribution.sample()
            log_prob = distribution.log_prob(actions).sum(dim=-1)
            values = self.policy.value(privileged, latent).squeeze(-1)
        return actions, log_prob, values, mean, std

    def value_of(self, privileged: torch.Tensor, history: torch.Tensor) -> torch.Tensor:
        with torch.no_grad():
            latent = self.policy.encode(history)
            return self.policy.value(privileged, latent).squeeze(-1)

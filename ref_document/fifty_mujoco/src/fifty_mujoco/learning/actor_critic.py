"""Networks for the Fudan-style sequence policy (doc section 7.3).

    encoder : 125 -> 128 -> 64 -> 3     (ELU)
    actor   : (25 + 3) -> 128 -> 64 -> 32 -> 6
    critic  : 152 -> 256 -> 128 -> 64 -> 1

The latent is *detached* for the policy loss: the encoder is trained only by the
simulation-truth velocity loss and, optionally, never by the teacher loss.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import nn


def _mlp(sizes: list[int], activation: type[nn.Module] = nn.ELU) -> nn.Sequential:
    layers: list[nn.Module] = []
    for index in range(len(sizes) - 1):
        layers.append(nn.Linear(sizes[index], sizes[index + 1]))
        if index < len(sizes) - 2:
            layers.append(activation())
    return nn.Sequential(*layers)


@dataclass
class NetworkDims:
    num_obs: int = 25
    num_history: int = 125
    latent_dim: int = 3
    num_actions: int = 6
    num_privileged: int = 149
    critic_hidden: tuple[int, ...] = (256, 128, 64)
    actor_hidden: tuple[int, ...] = (128, 64, 32)
    encoder_hidden: tuple[int, ...] = (128, 64)
    init_std: float = 0.5

    @property
    def critic_input_dim(self) -> int:
        return self.num_privileged + self.latent_dim


class ActorCriticSequence(nn.Module):
    def __init__(self, dims: NetworkDims):
        super().__init__()
        self.dims = dims
        self.encoder = _mlp([dims.num_history, *dims.encoder_hidden, dims.latent_dim])
        self.actor = _mlp([dims.num_obs + dims.latent_dim, *dims.actor_hidden, dims.num_actions])
        self.critic = _mlp([dims.critic_input_dim, *dims.critic_hidden, 1])
        self.std = nn.Parameter(torch.full((dims.num_actions,), dims.init_std))

    # ------------------------------------------------------------------ pieces
    def encode(self, history: torch.Tensor) -> torch.Tensor:
        return self.encoder(history)

    def act_inference(self, obs: torch.Tensor, history: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        latent = self.encode(history)
        mean = self.actor(torch.cat([obs, latent.detach()], dim=-1))
        return mean, latent

    def evaluate(self, obs: torch.Tensor, history: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        latent = self.encode(history)
        mean = self.actor(torch.cat([obs, latent.detach()], dim=-1))
        return mean, self.std.expand_as(mean), latent

    def value(self, privileged: torch.Tensor, latent: torch.Tensor) -> torch.Tensor:
        return self.critic(torch.cat([privileged, latent], dim=-1))

    # ------------------------------------------------------------------ checks
    def shapes(self) -> dict[str, tuple[int, ...]]:
        return {
            "encoder.0.weight": tuple(self.encoder[0].weight.shape),
            "actor.0.weight": tuple(self.actor[0].weight.shape),
            "critic.0.weight": tuple(self.critic[0].weight.shape),
        }

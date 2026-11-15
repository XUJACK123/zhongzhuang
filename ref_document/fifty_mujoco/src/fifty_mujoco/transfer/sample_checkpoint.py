"""Strict, independent 31-input feedforward sample actor (no history/encoder)."""

from __future__ import annotations

from pathlib import Path

import torch
from torch import nn

from ..contracts.sample_actor31 import ACTOR_SIZES


class SampleActor31(nn.Module):
    def __init__(self):
        super().__init__()
        layers = []
        for i, (a, b) in enumerate(zip(ACTOR_SIZES, ACTOR_SIZES[1:])):
            layers.append(nn.Linear(a, b))
            if i < len(ACTOR_SIZES) - 2:
                layers.append(nn.ELU())
        self.actor = nn.Sequential(*layers)

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        if observation.ndim != 2 or observation.shape[1] != 31:
            raise ValueError("sample actor requires [batch, 31] observations")
        if not torch.isfinite(observation).all():
            raise ValueError("non-finite sample observation")
        return self.actor(observation)


def load_sample_actor(path: str | Path, device: str = "cpu") -> tuple[SampleActor31, dict]:
    """Validate ALL source model tensors, then strictly restore the actor only.

    Critic/optimizer are not restored into a training experiment. weights_only
    prevents the checkpoint from requesting arbitrary Python deserialization.
    """
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    state = checkpoint["model_state_dict"]
    expected = {"std": (6,)}
    for prefix, sizes in (("actor", ACTOR_SIZES), ("critic", (147, 256, 128, 64, 1))):
        for i, (a, b) in enumerate(zip(sizes, sizes[1:])):
            expected[f"{prefix}.{2*i}.weight"] = (b, a)
            expected[f"{prefix}.{2*i}.bias"] = (b,)
    if set(state) != set(expected):
        raise ValueError(f"unexpected model schema; missing={sorted(set(expected)-set(state))}; "
                         f"extra={sorted(set(state)-set(expected))}")
    for name, shape in expected.items():
        tensor = state[name]
        if not isinstance(tensor, torch.Tensor) or tensor.dtype != torch.float32:
            raise ValueError(f"{name}: expected float32 tensor")
        if tuple(tensor.shape) != shape or not torch.isfinite(tensor).all():
            raise ValueError(f"{name}: wrong shape or non-finite parameters")
    if not (state["std"] > 0).all():
        raise ValueError("nonpositive source exploration standard deviation")
    actor = SampleActor31()
    actor.load_state_dict({k: v for k, v in state.items() if k.startswith("actor.")}, strict=True)
    actor.to(device).eval().requires_grad_(False)
    return actor, {"iteration": int(checkpoint["iter"]), "model_tensor_count": len(state),
                   "actor_parameters": sum(p.numel() for p in actor.parameters()),
                   "critic_restored": False, "optimizer_restored": False,
                   "full_training_resume": False}

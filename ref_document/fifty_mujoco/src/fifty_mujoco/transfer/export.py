"""P10: export the actor + encoder to ONNX and run an ONNX policy on the CPU."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import torch


class _PolicyWrapper(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, obs: torch.Tensor, obs_history: torch.Tensor) -> torch.Tensor:
        mean, _latent = self.policy.act_inference(obs, obs_history)
        return mean


def export_onnx(policy, path: str | Path, opset: int = 17, batch: int = 1) -> dict[str, Any]:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    wrapper = _PolicyWrapper(policy).eval()
    dims = policy.dims
    obs = torch.zeros(batch, dims.num_obs)
    history = torch.zeros(batch, dims.num_history)
    torch.onnx.export(
        wrapper,
        (obs, history),
        str(path),
        input_names=["obs", "obs_history"],
        output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "obs_history": {0: "batch"}, "actions": {0: "batch"}},
        opset_version=opset,
        do_constant_folding=True,
    )
    return {"path": str(path), "opset": opset, "batch": batch}


def verify_onnx(policy, path: str | Path, samples: int = 256, seed: int = 3) -> dict[str, Any]:
    import onnxruntime as ort

    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(seed)
    obs = rng.normal(0.0, 0.5, size=(samples, policy.dims.num_obs)).astype(np.float32)
    history = rng.normal(0.0, 0.5, size=(samples, policy.dims.num_history)).astype(np.float32)
    reference = session.run(["actions"], {"obs": obs, "obs_history": history})[0]
    with torch.no_grad():
        mean, _ = policy.act_inference(torch.as_tensor(obs), torch.as_tensor(history))
    restored = mean.numpy()
    absolute = float(np.max(np.abs(restored - reference)))
    return {
        "samples": samples,
        "max_abs_error": absolute,
        "passed": absolute <= 1e-5 + 1e-4 * float(np.max(np.abs(reference))),
    }


class OnnxPolicy:
    """Deterministic policy for playback: mean action, no exploration."""

    def __init__(self, path: str | Path):
        import onnxruntime as ort

        self.path = str(path)
        self.session = ort.InferenceSession(self.path, providers=["CPUExecutionProvider"])
        self.inputs = {item.name for item in self.session.get_inputs()}

    def act(self, obs: np.ndarray, history: np.ndarray) -> np.ndarray:
        return self.session.run(
            ["actions"],
            {"obs": np.asarray(obs, dtype=np.float32), "obs_history": np.asarray(history, dtype=np.float32)},
        )[0]


def write_export_manifest(policy, path: str | Path, extra: dict[str, Any] | None = None) -> Path:
    import hashlib
    import json

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "onnx": str(path),
        "onnx_sha256": hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None,
        "obs_dim": policy.dims.num_obs,
        "history_dim": policy.dims.num_history,
        "action_dim": policy.dims.num_actions,
        "critic_input_dim": policy.dims.critic_input_dim,
        "note": (
            "The ONNX contains actor+encoder only. Default joint angles, PD gains, action scales, "
            "spawn pose and gas-spring settings must be shipped next to it."
        ),
    }
    payload.update(extra or {})
    manifest = path.with_suffix(".manifest.json")
    manifest.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return manifest

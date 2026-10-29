"""P6: recover the Fudan teacher (encoder + actor) from ONNX and verify it.

The ONNX file only contains the encoder and the actor; the critic and the
optimisers do not exist in it and are never invented.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch

from ..learning.actor_critic import ActorCriticSequence, NetworkDims


@dataclass
class Teacher:
    policy: ActorCriticSequence
    std: torch.Tensor
    report: dict[str, Any]
    path: str


def _initializers(path: Path) -> dict[str, np.ndarray]:
    import onnx
    from onnx import numpy_helper

    model = onnx.load(str(path))
    return {tensor.name: numpy_helper.to_array(tensor) for tensor in model.graph.initializer}


def _match_parameters(module: torch.nn.Module, arrays: dict[str, np.ndarray], used: set[str]) -> int:
    """Greedy shape-ordered assignment; returns the number of tensors matched."""
    targets = list(module.named_parameters())
    names_in_order = list(arrays.keys())
    matched = 0
    cursor = 0
    for name, parameter in targets:
        shape = tuple(parameter.shape)
        found = None
        for index in range(cursor, len(names_in_order)):
            candidate = names_in_order[index]
            if candidate in used:
                continue
            if tuple(arrays[candidate].shape) == shape:
                found = candidate
                cursor = index + 1
                break
        if found is None:
            raise ValueError(f"could not match ONNX tensor for {name} with shape {shape}")
        parameter.data.copy_(torch.from_numpy(np.asarray(arrays[found])).to(parameter.dtype))
        used.add(found)
        matched += 1
    return matched


def restore_teacher(
    path: str | Path,
    dims: NetworkDims,
    device: str = "cpu",
    tolerance_atol: float = 1e-5,
    tolerance_rtol: float = 1e-4,
    samples: int = 256,
    seed: int = 7,
) -> Teacher:
    import onnxruntime as ort

    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"teacher ONNX not found: {path}")

    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    inputs = {item.name: item.shape for item in session.get_inputs()}
    outputs = [item.name for item in session.get_outputs()]

    obs_name = next((name for name, shape in inputs.items() if shape[-1] == dims.num_obs), None)
    history_name = next((name for name, shape in inputs.items() if shape[-1] == dims.num_history), None)
    if obs_name is None or history_name is None:
        raise ValueError(f"teacher inputs {inputs} do not match obs {dims.num_obs} / history {dims.num_history}")

    arrays = _initializers(path)
    policy = ActorCriticSequence(dims).to(device)
    used: set[str] = set()
    _match_parameters(policy.encoder, arrays, used)
    _match_parameters(policy.actor, arrays, used)

    # Numerical equivalence check: ONNX vs the restored PyTorch sub-network.
    rng = np.random.default_rng(seed)
    obs = rng.normal(0.0, 0.5, size=(samples, dims.num_obs)).astype(np.float32)
    history = rng.normal(0.0, 0.5, size=(samples, dims.num_history)).astype(np.float32)
    reference = session.run(outputs, {obs_name: obs, history_name: history})[0]

    with torch.no_grad():
        mean, latent = policy.act_inference(
            torch.as_tensor(obs, device=device), torch.as_tensor(history, device=device)
        )
    restored = mean.cpu().numpy()
    absolute = float(np.max(np.abs(restored - reference)))
    denominator = np.maximum(np.abs(reference), 1e-8)
    relative = float(np.max(np.abs(restored - reference) / denominator))

    report = {
        "path": str(path),
        "inputs": {name: list(shape) for name, shape in inputs.items()},
        "outputs": outputs,
        "matched_tensors": len(used),
        "unmatched_tensors": sorted(set(arrays) - used),
        "samples": samples,
        "max_abs_error": absolute,
        "max_rel_error": relative,
        "tolerances": {"atol": tolerance_atol, "rtol": tolerance_rtol},
        "passed": absolute <= tolerance_atol + tolerance_rtol * float(np.max(np.abs(reference))),
        "note": "critic and optimisers do not exist in the ONNX file and are not invented",
    }
    if not report["passed"]:
        raise ValueError(f"teacher restoration failed numerical check: {report}")

    policy.eval()
    for parameter in policy.parameters():
        parameter.requires_grad_(False)

    # The teacher's own exploration scale is not stored either; 0.1 matches the
    # lower clamp used by the teacher loss (doc 7.3).
    std = torch.full((dims.num_actions,), 0.1, device=device)
    return Teacher(policy=policy, std=std, report=report, path=str(path))


def write_teacher_manifest(teacher: Teacher, path: str | Path) -> Path:
    import hashlib
    import json

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "teacher": teacher.report,
        "weights_sha256": hashlib.sha256(Path(teacher.path).read_bytes()).hexdigest(),
        "frozen": True,
        "channel_order": None,
        "channel_contract_status": "unverified",
        "note": (
            "The teacher channel order is not proven by weight matching. "
            "`evaluate-teacher` must reproduce the original robot before the teacher loss is trusted."
        ),
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path

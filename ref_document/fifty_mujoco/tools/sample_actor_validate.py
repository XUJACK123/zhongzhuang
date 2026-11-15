#!/usr/bin/env python3
"""V1 fixed-input CPU / optional CUDA / recovered-source network comparison."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import importlib.util
import io
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch

PROJECT = Path(__file__).resolve().parents[1]
WORKSPACE = PROJECT.parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from fifty_mujoco.transfer.sample_checkpoint import load_sample_actor


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_actor_class():
    relative = "ref_document/fudan_rl_wheel_leg-main/plane/wheel_legged_gym/rsl_rl/modules/actor_critic.py"
    expected = json.loads((WORKSPACE / "sample_weight_file/source-files.json").read_text())["files"][relative]
    path = WORKSPACE / relative
    if digest(path) != expected["sha256"]:
        raise ValueError("recovered ActorCritic source does not match frozen training source hash")
    spec = importlib.util.spec_from_file_location("sample_verified_source_actor", path)
    module = importlib.util.module_from_spec(spec)
    exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
    return module.ActorCritic, {"path": relative, "sha256": digest(path), "hash_verified": True}


def numpy_forward(actor, inputs):
    result = inputs.numpy().astype(np.float64)
    for i in range(0, 8, 2):
        result = result @ actor.actor[i].weight.numpy().astype(np.float64).T
        result += actor.actor[i].bias.numpy().astype(np.float64)
        if i < 6:
            result = np.maximum(result, 0) + np.expm1(np.minimum(result, 0))
    return result


def validate(cuda=False):
    torch.set_num_threads(1)
    original_class, source = source_actor_class()
    if cuda and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable; CPU fallback is not GPU validation")
    batches = {}
    generator = torch.Generator().manual_seed(20261112)
    for n in (1, 32, 256):
        batches[f"random_{n}"] = torch.randn(n, 31, generator=generator) * .5
    batches["zero_1"] = torch.zeros(1, 31)
    batches["clip_endpoints_32"] = torch.linspace(-100, 100, 32 * 31).reshape(32, 31)
    reports = []
    for iteration in (2975, 3000, 3025, 3050):
        path = WORKSPACE / f"sample_weight_file/artifacts/model_{iteration}.pt"
        manifest = json.loads((WORKSPACE / "sample_weight_file/manifest.json").read_text())["files"]
        if digest(path) != manifest[f"artifacts/{path.name}"]["sha256"]:
            raise ValueError(f"checkpoint hash mismatch: {path.name}")
        actor, metadata = load_sample_actor(path)
        with contextlib.redirect_stdout(io.StringIO()):
            original = original_class(31, 147, 6, actor_hidden_dims=[128, 64, 32],
                                      critic_hidden_dims=[256, 128, 64], activation="elu")
        state = torch.load(path, map_location="cpu", weights_only=True)["model_state_dict"]
        original.load_state_dict(state, strict=True)
        original.eval().requires_grad_(False)
        gpu_actor = load_sample_actor(path, "cuda")[0] if cuda else None
        rows = []
        with torch.inference_mode():
            for name, inputs in batches.items():
                output = actor(inputs)
                reference = original.act_inference(inputs)
                independent = torch.from_numpy(numpy_forward(actor, inputs))
                comparisons = {"recovered_source_cpu": reference,
                               "numpy_float64": independent}
                if gpu_actor is not None:
                    comparisons["cuda"] = gpu_actor(inputs.cuda()).cpu()
                details = {}
                for key, other in comparisons.items():
                    actual = output.to(other.dtype)
                    details[key] = {"max_abs_error": float((actual-other).abs().max()),
                                    "passed": bool(torch.isfinite(other).all()
                                                   and torch.allclose(actual, other, atol=1e-5, rtol=1e-4))}
                rows.append({"input": name, "batch": len(inputs), "input_sha256":
                             hashlib.sha256(inputs.numpy().tobytes()).hexdigest(),
                             "output_shape": list(output.shape),
                             "all_finite": bool(torch.isfinite(output).all()),
                             "comparisons": details})
        passed = all(r["all_finite"] and all(v["passed"] for v in r["comparisons"].values()) for r in rows)
        reports.append({"checkpoint": path.name, "sha256": digest(path), **metadata,
                        "strict_load": True, "passed": passed, "batches": rows})
    return {"tool": "sample-actor31-validation", "schema": 1,
            "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "status": "passed" if all(r["passed"] for r in reports) else "failed",
            "atol": 1e-5, "rtol": 1e-4, "source_network": source,
            "environment": {"python": sys.version.split()[0], "torch": torch.__version__,
                            "numpy": np.__version__, "cuda_tested": cuda,
                            "gpu": torch.cuda.get_device_name(0) if cuda else None},
            "checkpoints": reports, "source_runtime_golden_samples_available": False,
            "physics_executed": False, "training_executed": False,
            "scope": "Fixed-input network parity only; original environment observation/reset timing remains unverified."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cuda", action="store_true")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    report = validate(args.cuda)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "cuda_tested": args.cuda, "out": str(args.out)}))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

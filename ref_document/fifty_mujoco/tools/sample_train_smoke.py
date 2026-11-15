#!/usr/bin/env python3
"""Bounded transfer-training smoke: pretrained actor, fresh critic/optimizers.

This is a source-reconstruction plumbing test, not target adaptation acceptance.
CPU MuJoCo advances physics; --device cuda places PPO and policy on the GPU.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from fifty_mujoco.envs.sample_env31 import ENV_SCHEMA, REWARD_SCHEMA, SampleEnv31
from fifty_mujoco.learning.checkpoint import save_checkpoint
from fifty_mujoco.learning.sample_ppo import TRAIN_SCHEMA, SamplePolicy31, SamplePPO
from fifty_mujoco.transfer.sample_checkpoint import load_sample_actor
from fifty_mujoco.transfer.sample_source import verified_artifact

COMMANDS = [(0, 0, .1), (.15, 0, .1), (-.05, 0, .1), (0, .3, .1), (0, -.3, .1)]


def fingerprint(env):
    files = ["envs/sample_env31.py", "learning/sample_ppo.py", "contracts/sample_actor31.py",
             "transfer/sample_source.py", "transfer/sample_checkpoint.py"]
    return {"environment": ENV_SCHEMA, "reward": REWARD_SCHEMA,
            "model_xml": hashlib.sha256(env.source.xml.encode()).hexdigest(),
            "code": {name: hashlib.sha256((PROJECT/"src/fifty_mujoco"/name).read_bytes()).hexdigest() for name in files},
            "episode_steps": env.max_steps}


def train(args):
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable; use a GPU-accessible process")
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    source_path = verified_artifact("model_3050.pt")
    policy = SamplePolicy31(source_path, args.device)
    ppo = SamplePPO(policy)
    envs = [SampleEnv31(episode_seconds=args.episode_seconds) for _ in range(args.envs)]
    contract = fingerprint(envs[0])
    start_update = 0
    origin_sha = hashlib.sha256(source_path.read_bytes()).hexdigest()
    if args.resume:
        checkpoint = torch.load(args.resume, map_location="cpu", weights_only=True)
        if checkpoint["schema"] != TRAIN_SCHEMA or checkpoint["contract"] != contract:
            raise ValueError("resume contract/model/code mismatch")
        if checkpoint["source_checkpoint_sha256"] != origin_sha:
            raise ValueError("source checkpoint changed since this experiment")
        policy.load_state_dict(checkpoint["model"], strict=True)
        ppo.actor_optimizer.load_state_dict(checkpoint["actor_optimizer"])
        ppo.critic_optimizer.load_state_dict(checkpoint["critic_optimizer"])
        torch.set_rng_state(checkpoint["torch_rng"].cpu())
        if args.device == "cuda" and checkpoint.get("cuda_rng"):
            torch.cuda.set_rng_state_all([state.cpu() for state in checkpoint["cuda_rng"]])
        start_update = checkpoint["update"]
    args.out.mkdir(parents=True, exist_ok=False)
    # Continuing the reconstructed PPO state resets simulators. This is
    # explicitly not a bit-identical continuation of simulator trajectories.
    observations = [env.reset(args.seed + 10000*start_update + i, COMMANDS[i % 5]) for i, env in enumerate(envs)]
    actor_before = {k: v.detach().cpu().clone() for k, v in policy.actor.state_dict().items()}
    initial = torch.tensor(np.stack(observations), device=args.device)
    with torch.no_grad():
        initial_actions = policy.actor(initial).cpu()
        restored, _ = load_sample_actor(source_path)
        source_error = float((initial_actions-restored(initial.cpu())).abs().max())
    rows = []
    episode_count = failure_count = 0
    completed = start_update
    started = time.monotonic()

    def save(update):
        payload = {"schema": TRAIN_SCHEMA, "contract": contract, "source_checkpoint_sha256": origin_sha,
            "model": policy.state_dict(), "actor_optimizer": ppo.actor_optimizer.state_dict(),
            "critic_optimizer": ppo.critic_optimizer.state_dict(), "update": update,
            "torch_rng": torch.get_rng_state(), "cuda_rng": torch.cuda.get_rng_state_all() if args.device == "cuda" else [],
            "resume_scope": "PPO_and_RNG_restored_simulators_reset", "source_critic_restored": False,
            "source_optimizer_restored": False}
        return save_checkpoint(args.out/f"update_{update:06d}.pt", payload)

    last_path = save(start_update)
    try:
        for update in range(start_update+1, start_update+args.updates+1):
            if time.monotonic()-started >= args.max_seconds:
                break
            collected = {name: [] for name in ("observation", "action", "log_prob", "value", "reward", "next_value", "terminated", "truncated", "mean", "std")}
            for _ in range(args.steps):
                obs = torch.tensor(np.stack(observations), device=args.device)
                with torch.no_grad():
                    distribution = policy.distribution(obs)
                    actions = distribution.sample()
                    values = policy.value(obs)
                    log_prob = distribution.log_prob(actions).sum(-1)
                results = [env.step(action) for env, action in zip(envs, actions.cpu().numpy())]
                terminal_obs = np.stack([row[0] for row in results])
                with torch.no_grad():
                    next_values = policy.value(torch.tensor(terminal_obs, device=args.device))
                rewards = torch.tensor([r[1] for r in results], device=args.device)
                terminated = torch.tensor([r[2] for r in results], device=args.device)
                truncated = torch.tensor([r[3] for r in results], device=args.device)
                for name, value in zip(collected, (obs, actions, log_prob, values, rewards, next_values,
                                                   terminated, truncated, distribution.mean, distribution.stddev)):
                    collected[name].append(value)
                observations = list(terminal_obs)
                for i, (_, _, terminal, timeout, _) in enumerate(results):
                    if terminal or timeout:
                        episode_count += 1
                        failure_count += int(terminal)
                        observations[i] = envs[i].reset(args.seed+10000*update+100*episode_count+i,
                            COMMANDS[(i+episode_count) % 5])
            warmup = update <= args.warmup_updates
            stats = ppo.update({k: torch.stack(v) for k, v in collected.items()}, warmup=warmup)
            if not all(math.isfinite(v) for v in stats.values()):
                raise FloatingPointError("nonfinite PPO statistics")
            if warmup and any(not torch.equal(v.cpu(), actor_before[k]) for k, v in policy.actor.state_dict().items()):
                raise AssertionError("critic warmup changed pretrained actor")
            completed = update
            last_path = save(completed)
            rows.append({"update": update, **stats, "episodes_ended": episode_count,
                         "failures": failure_count, "elapsed_s": time.monotonic()-started})
            print(json.dumps(rows[-1]), flush=True)
    except Exception as exc:
        # Keep the last completed atomic checkpoint, never a partially failed
        # or nonfinite optimizer step. Preserve the reason for review.
        (args.out/"failure.json").write_text(json.dumps({"status": "failed",
            "error": f"{type(exc).__name__}: {exc}", "last_completed_update": completed,
            "last_good_checkpoint": str(last_path.resolve())}, indent=2)+"\n")
        raise
    # Reload all training state, then compare inference before accepting save.
    saved = torch.load(last_path, weights_only=True, map_location="cpu")
    check = SamplePolicy31(source_path).eval()
    check.load_state_dict(saved["model"], strict=True)
    with torch.no_grad():
        roundtrip_error = float((check.actor(initial.cpu())-policy.actor(initial).cpu()).abs().max())
    report = {"tool": "sample-transfer-training-smoke", "schema": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": "passed" if completed > start_update and roundtrip_error < 1e-5 else "failed",
        "training_schema": TRAIN_SCHEMA, "contract": contract, "device": args.device,
        "gpu": torch.cuda.get_device_name(0) if args.device == "cuda" else None,
        "physics_backend": "CPU_MuJoCo", "start_update": start_update, "last_update": completed,
        "transitions": (completed-start_update)*args.steps*args.envs,
        "source_checkpoint_sha256": origin_sha, "source_actor_initial_max_abs_error": source_error,
        "source_critic_restored": False, "source_optimizer_restored": False,
        "checkpoint": str(last_path.resolve()), "checkpoint_sha256": hashlib.sha256(last_path.read_bytes()).hexdigest(),
        "checkpoint_inference_roundtrip_max_abs_error": roundtrip_error,
        "actor_max_parameter_delta": max(float((v.cpu()-actor_before[k]).abs().max()) for k, v in policy.actor.state_dict().items()),
        "episodes_ended": episode_count, "failures": failure_count, "rows": rows,
        "randomization_enabled": False, "target_model_accepted": False,
        "scope": "Source reconstruction training plumbing only; no motion/robustness acceptance.",
        "resume": str(args.resume) if args.resume else None,
        "resume_scope": "PPO_and_RNG_restored_simulators_reset"}
    (args.out/"smoke_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--updates", type=int, default=4)
    parser.add_argument("--warmup-updates", type=int, default=2)
    parser.add_argument("--envs", type=int, default=8)
    parser.add_argument("--steps", type=int, default=32)
    parser.add_argument("--episode-seconds", type=float, default=.5)
    parser.add_argument("--max-seconds", type=float, default=60)
    parser.add_argument("--seed", type=int, default=20261112)
    args = parser.parse_args()
    if not (1 <= args.updates <= 20 and 1 <= args.envs <= 32 and 1 <= args.steps <= 128):
        parser.error("smoke bounds: updates1..20, envs1..32, steps1..128")
    if not (0 <= args.warmup_updates <= 20 and math.isfinite(args.max_seconds) and 0 < args.max_seconds <= 120):
        parser.error("warmup0..20, wall budget0..120s required")
    report = train(args)
    print(json.dumps({k: report[k] for k in ("status", "last_update", "transitions", "checkpoint", "actor_max_parameter_delta")}))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

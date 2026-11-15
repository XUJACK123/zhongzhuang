#!/usr/bin/env python3
"""Wall-bounded training on the commissioned hardware reconstruction.

Unlike tools/sample_train_smoke.py this entry is not capped at 20 updates or
120 s. It is still a *source-reconstruction* transfer experiment: the archived
actor is reused, the critic and both optimizers are new, physics is CPU MuJoCo. Mixed resets span the default and surveyed natural pose. Nothing here accepts the target robot.

Segments are wall-clock bounded (default 1 h) and fully resumable through this
tool's own checkpoints (model + both optimizers + RNG + update index).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
import signal
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from fifty_mujoco.envs.sample_env31 import wilson_interval
from fifty_mujoco.envs.sample_env31_hardware import ENV_SCHEMA, REWARD_SCHEMA
from fifty_mujoco.envs.sample_env31_hardware_fast import FastHardwareEnv31 as SampleEnv31
from fifty_mujoco.learning.checkpoint import save_checkpoint
from fifty_mujoco.learning.sample_ppo import TRAIN_SCHEMA, SamplePolicy31, SamplePPO
from fifty_mujoco.transfer.sample_checkpoint import load_sample_actor
from fifty_mujoco.transfer.sample_source import verified_artifact

TRAIN_COMMANDS = [(0, 0, .1), (.15, 0, .1), (-.15, 0, .1), (0, .3, .1), (0, -.3, .1)]
EVAL_CLASSES = [("stand", (0, 0, .1)), ("forward", (.15, 0, .1)), ("backward", (-.15, 0, .1)),
                ("left", (0, .3, .1)), ("right", (0, -.3, .1))]
# Separate reconstructed-development seeds; never the heldout sets in the spec.
EVAL_SEED_BASE = 91000


def fingerprint(env):
    files = ["envs/sample_env31.py", "learning/sample_ppo.py", "contracts/sample_actor31.py",
             "transfer/sample_source.py", "transfer/sample_checkpoint.py",
             "envs/sample_env31_hardware.py", "envs/sample_env31_hardware_fast.py", "transfer/sample_target.py", "envs/sample_env31_target.py"]
    return {"environment": ENV_SCHEMA, "reward": REWARD_SCHEMA, "actuator_backend": "scalar_float64",
            "model_xml": hashlib.sha256(env.source.xml.encode()).hexdigest(),
            "code": {name: hashlib.sha256((PROJECT / "src/fifty_mujoco" / name).read_bytes()).hexdigest()
                     for name in files},
            "episode_steps": env.max_steps, "hardware_profile": env.source.profile,
            "reset_mode": env.reset_mode, "motor_gain": env.motor_gain.tolist(), "spring_scale": env.spring_scale, "wheel_kd": env.wheel_kd,
            "trainer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def thresholds():
    return json.loads((PROJECT / "configs/sample_command_evaluation.json").read_text())["thresholds"]


def evaluate(policy, args):
    """Deterministic mean-action rollouts on the five command classes."""
    limit = thresholds()
    rows, summaries = [], {}
    for index, (name, command) in enumerate(EVAL_CLASSES):
        current = []
        for j in range(args.eval_seeds):
            env = SampleEnv31(episode_seconds=args.eval_seconds, effective_friction=args.friction,
                              reset_mode=args.eval_reset_mode, motor_gain=args.motor_gain, spring_scale=args.spring_scale, wheel_kd=args.wheel_kd)
            obs = env.reset(EVAL_SEED_BASE + index * 100 + j, command)
            samples, saturation = [], np.zeros(6)
            controls, minimum_margin = 0, math.inf
            info = None
            for _ in range(env.max_steps):
                with torch.no_grad():
                    action = policy.actor(torch.tensor(obs, device=args.device)[None])[0].cpu().numpy()
                obs, _, terminated, truncated, info = env.step(action)
                applied = info["applied_command"]
                saturation += info["saturation_counts"]
                controls += info["controls"]
                if info["spring_margins_m"]:
                    minimum_margin = min(minimum_margin, *info["spring_margins_m"])
                if info["time_s"] >= 2:
                    samples.append([info["point_velocity_xy"][0] - applied[0],
                                    info["point_velocity_xy"][1],
                                    info["yaw_rate"] - applied[1],
                                    abs(info["clearance_m"] - applied[2]),
                                    *np.abs(info["roll_pitch_deg"])])
                if terminated or truncated:
                    break
            metrics = {"samples": len(samples)}
            if samples:
                values = np.asarray(samples)
                metrics.update(
                    point_vx_rmse_m_s=float(np.sqrt(np.mean(values[:, 0] ** 2))),
                    point_vy_rmse_m_s=float(np.sqrt(np.mean(values[:, 1] ** 2))),
                    yaw_rate_rmse_rad_s=float(np.sqrt(np.mean(values[:, 2] ** 2))),
                    clearance_mae_m=float(values[:, 3].mean()),
                    roll_abs_p95_deg=float(np.percentile(values[:, 4], 95)),
                    pitch_abs_p95_deg=float(np.percentile(values[:, 5], 95)))
            met = bool(truncated and not terminated and samples
                       and metrics["point_vx_rmse_m_s"] <= limit["vx_rmse_max_m_s"]
                       and metrics["yaw_rate_rmse_rad_s"] <= limit["wz_rmse_max_rad_s"]
                       and metrics["clearance_mae_m"] <= limit["clearance_mae_max_m"]
                       and max(metrics["roll_abs_p95_deg"], metrics["pitch_abs_p95_deg"])
                       <= limit["tilt_p95_max_deg"]
                       and (saturation / max(1, controls)).max()
                       <= limit["motor_saturation_max_per_motor"])
            current.append({"scenario": name, "seed": EVAL_SEED_BASE + index * 100 + j,
                            "duration_s": args.eval_seconds, "completed": bool(truncated and not terminated),
                            "first_failure": info["failure"] if info else None,
                            "thresholds_met": met, "metrics": metrics,
                            "motor_saturation_fractions": (saturation / max(1, controls)).tolist(),
                            "minimum_spring_margin_m": minimum_margin if math.isfinite(minimum_margin) else None})
        wins = sum(row["thresholds_met"] for row in current)
        summaries[name] = {"trials": len(current), "thresholds_met": wins,
                           "rate": wins / len(current), "wilson_95": wilson_interval(wins, len(current))}
        rows.extend(current)
        print(json.dumps({"eval_class": name, **summaries[name]}), flush=True)
    passed = all(summary["rate"] >= thresholds()["per_class_rate_min"] for summary in summaries.values())
    return {"rows": rows, "summaries": summaries, "all_classes_threshold_met": passed}


def train(args):
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable; use a GPU-accessible process")
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    source_path = verified_artifact("model_3050.pt")
    policy = SamplePolicy31(source_path, args.device)
    ppo = SamplePPO(policy, actor_lr=args.actor_lr, critic_lr=args.critic_lr)
    envs = [SampleEnv31(episode_seconds=args.episode_seconds, effective_friction=args.friction,
                         reset_mode=args.reset_mode, motor_gain=args.motor_gain, spring_scale=args.spring_scale, wheel_kd=args.wheel_kd)
            for _ in range(args.envs)]
    if args.init_checkpoint:
        initial_checkpoint = torch.load(args.init_checkpoint, map_location="cpu", weights_only=True)
        policy.load_state_dict(initial_checkpoint["model"], strict=True)
    contract = fingerprint(envs[0])
    hyperparameters = {"actor_lr": args.actor_lr, "critic_lr": args.critic_lr,
                       "epochs": args.epochs, "batch_size": args.batch_size,
                       "warmup_updates": args.warmup_updates, "envs": args.envs,
                       "steps": args.steps, "episode_seconds": args.episode_seconds}
    origin_sha = hashlib.sha256(source_path.read_bytes()).hexdigest()
    start_update = 0
    if args.resume:
        checkpoint = torch.load(args.resume, map_location="cpu", weights_only=True)
        if checkpoint["schema"] != TRAIN_SCHEMA or checkpoint["contract"] != contract:
            raise ValueError("resume contract/model/code mismatch")
        if checkpoint["source_checkpoint_sha256"] != origin_sha:
            raise ValueError("source checkpoint changed since this experiment")
        if checkpoint.get("hyperparameters") != hyperparameters:
            raise ValueError("resume hyperparameter mismatch")
        policy.load_state_dict(checkpoint["model"], strict=True)
        ppo.actor_optimizer.load_state_dict(checkpoint["actor_optimizer"])
        ppo.critic_optimizer.load_state_dict(checkpoint["critic_optimizer"])
        torch.set_rng_state(checkpoint["torch_rng"].cpu())
        if args.device == "cuda" and checkpoint.get("cuda_rng"):
            torch.cuda.set_rng_state_all([state.cpu() for state in checkpoint["cuda_rng"]])
        start_update = checkpoint["update"]
    args.out.mkdir(parents=True, exist_ok=False)
    observations = [env.reset(args.seed + 10000 * start_update + i, TRAIN_COMMANDS[i % 5])
                    for i, env in enumerate(envs)]
    actor_before = {k: v.detach().cpu().clone() for k, v in policy.actor.state_dict().items()}
    initial = torch.tensor(np.stack(observations), device=args.device)
    with torch.no_grad():
        initial_actions = policy.actor(initial).cpu()
        restored, _ = load_sample_actor(source_path)
        source_error = float((initial_actions - restored(initial.cpu())).abs().max())
    settings = {"schema": TRAIN_SCHEMA, "contract": contract, "hyperparameters": hyperparameters,
                "source_checkpoint_sha256": origin_sha,
                "initialization_checkpoint": str(args.init_checkpoint) if args.init_checkpoint else None,
                "initialization_sha256": hashlib.sha256(args.init_checkpoint.read_bytes()).hexdigest() if args.init_checkpoint else None,
                "resume_scope": "PPO_and_RNG_restored_simulators_reset",
                "source_critic_restored": False, "source_optimizer_restored": False}

    def save(update):
        payload = {**settings, "model": policy.state_dict(),
                   "actor_optimizer": ppo.actor_optimizer.state_dict(),
                   "critic_optimizer": ppo.critic_optimizer.state_dict(), "update": update,
                   "torch_rng": torch.get_rng_state(),
                   "cuda_rng": torch.cuda.get_rng_state_all() if args.device == "cuda" else []}
        return save_checkpoint(args.out / f"update_{update:06d}.pt", payload)

    rows, evaluations = [], []
    stop_requested = False
    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    episode_count = failure_count = 0
    window_returns, window_lengths = [], []
    running_return = [0.0] * args.envs
    running_length = [0] * args.envs
    completed = start_update
    started = time.monotonic()
    last_path = save(start_update)
    history = (args.out / "rows.jsonl").open("a", encoding="utf-8")
    print(json.dumps({"start_update": start_update, "envs": args.envs, "steps": args.steps,
                      "rollout": args.envs * args.steps, "device": args.device,
                      "max_seconds": args.max_seconds}), flush=True)
    stop_reason = "update_limit"
    try:
        for update in range(start_update + 1, start_update + args.updates + 1):
            if stop_requested or time.monotonic() - started >= args.max_seconds - 15 or time.time() >= args.deadline_epoch - 15:
                stop_reason = "requested_stop" if stop_requested else "wall_budget"
                break
            segment_started = time.monotonic()
            collected = {name: [] for name in ("observation", "action", "log_prob", "value",
                                               "reward", "next_value", "terminated", "truncated",
                                               "mean", "std")}
            window_returns.clear()
            window_lengths.clear()
            window_failures = 0
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
                for name, value in zip(collected, (obs, actions, log_prob, values, rewards,
                                                   next_values, terminated, truncated,
                                                   distribution.mean, distribution.stddev)):
                    collected[name].append(value)
                observations = list(terminal_obs)
                for i, (_, reward, terminal, timeout, _) in enumerate(results):
                    running_return[i] += reward
                    running_length[i] += 1
                    if terminal or timeout:
                        episode_count += 1
                        failure_count += int(terminal)
                        window_failures += int(terminal)
                        window_returns.append(running_return[i])
                        window_lengths.append(running_length[i])
                        running_return[i], running_length[i] = 0.0, 0
                        observations[i] = envs[i].reset(
                            args.seed + 10000 * update + 100 * episode_count + i,
                            TRAIN_COMMANDS[(i + episode_count) % 5])
            warmup = update <= args.warmup_updates
            stats = ppo.update({k: torch.stack(v) for k, v in collected.items()}, warmup=warmup,
                               epochs=args.epochs, batch_size=args.batch_size)
            if not all(math.isfinite(v) for v in stats.values()):
                raise FloatingPointError("nonfinite PPO statistics")
            if warmup and any(not torch.equal(v.cpu(), actor_before[k])
                              for k, v in policy.actor.state_dict().items()):
                raise AssertionError("critic warmup changed pretrained actor")
            completed = update
            elapsed = time.monotonic() - started
            row = {"update": update, **stats, "episodes_ended": episode_count,
                   "window_episodes": len(window_returns), "window_failures": window_failures,
                   "window_return_mean": float(np.mean(window_returns)) if window_returns else None,
                   "window_length_mean": float(np.mean(window_lengths)) if window_lengths else None,
                   "elapsed_s": elapsed,
                   "segment_s": time.monotonic() - segment_started,
                   "throughput_samples_per_s": (update - start_update) * args.envs * args.steps / elapsed}
            rows.append(row)
            history.write(json.dumps(row) + "\n")
            history.flush()
            print(json.dumps(row), flush=True)
            if args.save_every and update % args.save_every == 0:
                last_path = save(completed)
            if (args.eval_every and update % args.eval_every == 0
                    and time.monotonic() - started < args.max_seconds - 90
                    and time.time() < args.deadline_epoch - 90):
                evaluation = {"update": update, "elapsed_s": time.monotonic() - started,
                              **evaluate(policy, args)}
                evaluation["score"] = float(np.mean([
                    row["metrics"].get("point_vx_rmse_m_s", 1.)/.1
                    + row["metrics"].get("yaw_rate_rmse_rad_s", 1.)/.15
                    + row["metrics"].get("clearance_mae_m", 1.)/.02
                    for row in evaluation["rows"]]))
                evaluations.append(evaluation)
                (args.out / "evaluations.jsonl").open("a", encoding="utf-8").write(
                    json.dumps(evaluation) + "\n")
            if args.plateau_evals and len(evaluations) >= args.plateau_evals:
                window = evaluations[-args.plateau_evals:]
                improvement = (window[0]["score"] - min(e["score"] for e in window[1:])) / max(window[0]["score"], 1e-9)
                if all(e["all_classes_threshold_met"] for e in window) and improvement < args.plateau_relative:
                    stop_reason = "development_plateau"
                    break
            if time.monotonic() - started >= args.max_seconds:
                stop_reason = "wall_budget"
                break
    except KeyboardInterrupt:
        history.close()
        (args.out / "interrupted.json").write_text(json.dumps(
            {"status": "interrupted", "last_completed_update": completed,
             "last_good_checkpoint": str(Path(last_path).resolve())}, indent=2) + "\n")
        raise
    except Exception as exc:
        history.close()
        (args.out / "failure.json").write_text(json.dumps(
            {"status": "failed", "error": f"{type(exc).__name__}: {exc}",
             "last_completed_update": completed,
             "last_good_checkpoint": str(Path(last_path).resolve())}, indent=2) + "\n")
        raise
    history.close()
    last_path = save(completed)
    saved = torch.load(last_path, weights_only=True, map_location="cpu")
    check = SamplePolicy31(source_path).eval()
    check.load_state_dict(saved["model"], strict=True)
    with torch.no_grad():
        roundtrip_error = float((check.actor(initial.cpu()) - policy.actor(initial).cpu()).abs().max())
    report = {"tool": "sample-transfer-training", "schema": 1,
              "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
              "status": "completed" if completed > start_update and roundtrip_error < 1e-5 else "failed",
              "training_schema": TRAIN_SCHEMA, "contract": contract, "hyperparameters": hyperparameters,
              "device": args.device,
              "gpu": torch.cuda.get_device_name(0) if args.device == "cuda" else None,
              "physics_backend": "CPU_MuJoCo", "start_update": start_update, "last_update": completed,
              "transitions": (completed - start_update) * args.steps * args.envs,
              "wall_seconds": time.monotonic() - started, "stop_reason": stop_reason,
              "source_checkpoint_sha256": origin_sha,
              "source_actor_initial_max_abs_error": source_error,
              "checkpoint": str(Path(last_path).resolve()),
              "checkpoint_sha256": hashlib.sha256(Path(last_path).read_bytes()).hexdigest(),
              "checkpoint_inference_roundtrip_max_abs_error": roundtrip_error,
              "actor_max_parameter_delta": max(float((v.cpu() - actor_before[k]).abs().max())
                                               for k, v in policy.actor.state_dict().items()),
              "episodes_ended": episode_count, "failures": failure_count,
              "evaluations": evaluations, "rows": rows,
              "randomization_enabled": False, "target_model_accepted": False,
              "scope": "Commissioned hardware reconstruction; not real robot acceptance.",
              "resume": str(args.resume) if args.resume else None}
    (args.out / "training_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--init-checkpoint", type=Path)
    parser.add_argument("--reset-mode", choices=["default", "natural", "mixed"], default="mixed")
    parser.add_argument("--eval-reset-mode", choices=["default", "natural"], default="natural")
    parser.add_argument("--wheel-kd", type=float, default=.3)
    parser.add_argument("--leg-gain", type=float, help="DM-only delivered torque gain; wheels remain at unity")
    parser.add_argument("--motor-gain", type=float, default=1.)
    parser.add_argument("--spring-scale", type=float, default=1.)
    parser.add_argument("--deadline-epoch", type=float, required=True)
    parser.add_argument("--plateau-evals", type=int, default=0)
    parser.add_argument("--plateau-relative", type=float, default=.01)
    parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda")
    parser.add_argument("--envs", type=int, default=32)
    parser.add_argument("--steps", type=int, default=32)
    parser.add_argument("--updates", type=int, default=1000000)
    parser.add_argument("--warmup-updates", type=int, default=2)
    parser.add_argument("--episode-seconds", type=float, default=20.)
    parser.add_argument("--friction", type=float, default=.75)
    parser.add_argument("--actor-lr", type=float, default=1e-6)
    parser.add_argument("--critic-lr", type=float, default=3e-4)
    parser.add_argument("--epochs", type=int, default=2)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--save-every", type=int, default=20)
    parser.add_argument("--eval-every", type=int, default=20)
    parser.add_argument("--eval-seconds", type=float, default=10.)
    parser.add_argument("--eval-seeds", type=int, default=1)
    parser.add_argument("--max-seconds", type=float, default=3600.)
    parser.add_argument("--seed", type=int, default=20261112)
    args = parser.parse_args()
    if args.leg_gain is not None:
        args.motor_gain = [args.leg_gain,args.leg_gain,1.,args.leg_gain,args.leg_gain,1.]
    if args.plateau_evals and args.plateau_evals < 3:
        parser.error("plateau requires at least three evaluations")
    if args.resume and args.init_checkpoint:
        parser.error("resume and initialization are mutually exclusive")
    if not math.isfinite(args.deadline_epoch) or args.max_seconds > 14400:
        parser.error("finite absolute deadline and segment <=4h required")
    if not (1 <= args.envs <= 64 and 1 <= args.steps <= 256 and args.updates >= 1):
        parser.error("envs1..64, steps1..256, updates>=1 required")
    if not (0 <= args.warmup_updates <= 50 and math.isfinite(args.max_seconds) and args.max_seconds > 0):
        parser.error("warmup0..50 and positive finite wall budget required")
    if not (math.isfinite(args.episode_seconds) and args.episode_seconds > 0):
        parser.error("positive finite episode duration required")
    if args.eval_every and not (math.isfinite(args.eval_seconds) and args.eval_seconds >= 2.01):
        parser.error("eval duration must exceed the 2 s tracking warmup")
    report = train(args)
    print(json.dumps({k: report[k] for k in ("status", "last_update", "transitions", "checkpoint",
                                             "actor_max_parameter_delta")}))
    return 0 if report["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

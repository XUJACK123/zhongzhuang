"""Independent standing evaluation (doc sections 5.4 and 14.2).

What the old ``trainer.evaluate`` could not answer
-------------------------------------------------
64 environments started from the same pose, the ``qpos``/velocity statistics
were read *after* the automatic reset, and a 6 s window counted as "survived".
A policy could therefore look good by standing up again and again from the
spawn pose.

This module evaluates a policy the way the acceptance table requires:

* one scenario per environment, distinct and reproducible seeds (default
  1000-1063, deliberately disjoint from the training seeds),
* a small but real initial disturbance (pitch/roll <= 2 deg, leg angle
  <= 0.02 rad, horizontal velocity <= 0.05 m/s) that is verified to be
  feasible (no ground penetration) instead of being silently dropped,
* 6000 policy steps = 60 s with the environment episode limit raised above the
  window, so a timeout can never be mistaken for survival,
* every statistic is accumulated **before** the reset of that step; the spawn
  state after a fall is never counted,
* per-episode results (survival time, reason, errors, saturation, drift) are
  reported next to the aggregate, failures included.

``up_min`` is reported twice on purpose: ``tilt_max_deg`` is the worst single
sample of any episode, ``tilt_p95_deg`` is the 95th percentile; they are not
the same number and must not be quoted interchangeably.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Protocol, Sequence

import mujoco
import numpy as np

from ..config import Config
from ..contracts import names
from ..envs.fifty_env import EnvOptions, FiftyVecEnv
from ..mdp.commands import CommandRanges

#: Policy callable protocol: ``(obs[N,25], history[N,125]) -> action[N,6]``.
PolicyFn = Callable[[np.ndarray, np.ndarray], np.ndarray]


class StandPolicy(Protocol):  # pragma: no cover - structural typing only
    def __call__(self, obs: np.ndarray, history: np.ndarray) -> np.ndarray: ...


def zero_policy(obs: np.ndarray, history: np.ndarray) -> np.ndarray:
    """Hold the spawn joint angles: useful as a control, not a real controller."""
    return np.zeros((obs.shape[0], names.NUM_ACTIONS))


@dataclass(frozen=True)
class Scenario:
    seed: int
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    leg_delta_rad: float = 0.0
    vx_m_s: float = 0.0

    def as_dict(self) -> dict[str, float]:
        return {
            "seed": self.seed,
            "pitch_deg": self.pitch_deg,
            "roll_deg": self.roll_deg,
            "leg_delta_rad": self.leg_delta_rad,
            "vx_m_s": self.vx_m_s,
        }


def make_scenarios(
    base_seed: int = 1000,
    count: int = 64,
    *,
    pitch_deg: float = 2.0,
    roll_deg: float = 2.0,
    leg_delta_rad: float = 0.02,
    vx_m_s: float = 0.05,
) -> list[Scenario]:
    """Independent, reproducible disturbances, one per environment.

    A zero disturbance is *not* injected implicitly: if a scenario draws a
    small value it is because that is what the contract allows.
    """
    scenarios: list[Scenario] = []
    for index in range(count):
        rng = np.random.default_rng(base_seed + index)
        scenarios.append(
            Scenario(
                seed=base_seed + index,
                pitch_deg=float(rng.uniform(-pitch_deg, pitch_deg)),
                roll_deg=float(rng.uniform(-roll_deg, roll_deg)),
                leg_delta_rad=float(rng.uniform(-leg_delta_rad, leg_delta_rad)),
                vx_m_s=float(rng.uniform(-vx_m_s, vx_m_s)),
            )
        )
    return scenarios


@dataclass
class EpisodeResult:
    seed: int
    survived_s: float
    survived_steps: int
    success: bool
    reason: str
    height_mae: float
    height_final: float
    vx_mae: float
    wz_mae: float
    tilt_p95_deg: float
    tilt_max_deg: float
    torque_saturation_fraction: float
    longest_saturation_s: float
    drift_m: float
    scenario: Scenario | None = None

    def as_dict(self) -> dict[str, Any]:
        payload = {
            "seed": self.seed,
            "survived_s": self.survived_s,
            "survived_steps": self.survived_steps,
            "success": self.success,
            "reason": self.reason,
            "height_mae_m": self.height_mae,
            "height_final_m": self.height_final,
            "vx_mae_m_s": self.vx_mae,
            "wz_mae_rad_s": self.wz_mae,
            "tilt_p95_deg": self.tilt_p95_deg,
            "tilt_max_deg": self.tilt_max_deg,
            "torque_saturation_fraction": self.torque_saturation_fraction,
            "longest_saturation_s": self.longest_saturation_s,
            "drift_m": self.drift_m,
        }
        if self.scenario is not None:
            payload["scenario"] = self.scenario.as_dict()
        return payload


#: First-version acceptance thresholds (doc 14.2).  Reported, never assumed.
THRESHOLDS: dict[str, float] = {
    "success_rate_min": 0.95,
    "vx_mae_max": 0.03,
    "height_mae_max": 0.02,
    "tilt_p95_max_deg": 10.0,
    "tilt_max_max_deg": 15.0,
    "torque_saturation_max": 0.05,
}


def _apply_scenario(env: FiftyVecEnv, index: int, scenario: Scenario) -> dict[str, Any]:
    """Set the perturbed initial state and verify it is physically feasible."""
    return _apply_scenario_to_data(env.model, env.data[index], env.single, scenario)


def _apply_scenario_to_data(model, data, single, scenario: Scenario) -> dict[str, Any]:
    """Same disturbance, applied to one ``MjData``.

    Kept separate from :func:`_apply_scenario` because the GPU evaluator seeds a
    scratch CPU ``MjData`` with the scenario (including the penetration check,
    which needs collision detection) and only then copies the result into the
    warp state.
    """
    quat = np.asarray(data.qpos[3:7], dtype=float)
    pitch = np.radians(scenario.pitch_deg)
    roll = np.radians(scenario.roll_deg)
    if pitch or roll:
        # small body-frame rotation composed on the right of the spawn quaternion
        cy, sy = np.cos(roll / 2.0), np.sin(roll / 2.0)
        cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
        roll_quat = np.array([cy, sy, 0.0, 0.0])
        pitch_quat = np.array([cp, 0.0, 0.0, sp])
        quat = _quat_mul(_quat_mul(quat, pitch_quat), roll_quat)
        data.qpos[3:7] = quat / np.linalg.norm(quat)

    if scenario.leg_delta_rad:
        for joint_name in names.LEG_JOINTS:
            data.qpos[single.qpos_adr[joint_name]] += scenario.leg_delta_rad

    if scenario.vx_m_s:
        data.qvel[0] = scenario.vx_m_s  # free joint: world +X

    mujoco.mj_forward(model, data)
    penetration = 0.0
    for contact_index in range(int(data.ncon)):
        distance = float(data.contact[contact_index].dist)
        if distance < 0.0:
            penetration = max(penetration, -distance)
    if penetration > 0.0:
        # Lift the robot just enough to remove the overlap; the disturbance is
        # kept, the infeasible part is not.
        data.qpos[2] += penetration + 1e-4
        mujoco.mj_forward(model, data)
    return {
        "penetration_fixed_m": penetration,
        "root_z": float(data.qpos[2]),
    }


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, wxyz, matching MuJoCo's convention."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def run_stand_eval(
    config: Config,
    policy: PolicyFn,
    *,
    steps: int = 6000,
    scenarios: Sequence[Scenario] | None = None,
    command: tuple[float, float, float] | None = None,
    settle_s: float = 1.0,
    torque_epsilon: float = 1e-3,
    backend: str = "cpu",
    device: str = "cuda:0",
) -> dict[str, Any]:
    """Run the acceptance window on one of the two physics backends.

    ``cpu`` is the reference implementation: one ``MjData`` per scenario, per-env
    reason strings, and an independent physics engine from training.

    ``warp`` runs the identical window on ``WarpVecEnv``. It is roughly 200x
    faster (64 scenarios x 6000 steps: 6.6 s against ~25 min), which is what
    makes the 60 s gate affordable after every training run. The two must agree;
    ``tests/test_stand_eval.py`` pins that on a short window.
    """
    chosen = list(scenarios or make_scenarios())
    if backend == "cpu":
        return _run_stand_eval_cpu(
            config,
            policy,
            steps=steps,
            scenarios=chosen,
            command=command,
            settle_s=settle_s,
            torque_epsilon=torque_epsilon,
        )
    if backend == "warp":
        return _run_stand_eval_warp(
            config,
            policy,
            steps=steps,
            scenarios=chosen,
            command=command,
            settle_s=settle_s,
            torque_epsilon=torque_epsilon,
            device=device,
        )
    raise ValueError(f"unknown backend {backend!r}; expected 'cpu' or 'warp'")


def _run_stand_eval_cpu(
    config: Config,
    policy: PolicyFn,
    *,
    steps: int = 6000,
    scenarios: Sequence[Scenario] | None = None,
    command: tuple[float, float, float] | None = None,
    settle_s: float = 1.0,
    torque_epsilon: float = 1e-3,
) -> dict[str, Any]:
    """Run one continuous episode per scenario and summarise the outcomes."""
    scenarios = list(scenarios or make_scenarios())
    env = FiftyVecEnv(
        config,
        EnvOptions(
            num_envs=len(scenarios),
            seed=scenarios[0].seed if scenarios else 0,
            episode_length_s=1e9,  # set below, once dt and the window are known
            # evaluation must hold one command for the whole window
            command_ranges=CommandRanges(resample_time_s=1e9),
        ),
    )
    dt = env.dt
    height = env.nominal_height if command is None else float(command[2])
    # The episode limit must sit *above* the acceptance window, otherwise a
    # 20 s environment timeout could masquerade as a 60 s survival.
    episode_length_s = steps * dt + max(1.0, steps * dt * 0.05)
    env.termination.episode_length_s = episode_length_s

    # A teacher is allowed to see the simulator state; a deployed policy is not.
    # ``bind`` is the only hook, and ``reset`` keeps its internal state aligned
    # with the new episode.
    if hasattr(policy, "bind"):
        policy.bind(env)
    if hasattr(policy, "reset"):
        policy.reset()

    n = env.num_envs
    env.reset()
    env.set_commands(
        np.array([0.0 if command is None else command[0], 0.0 if command is None else command[1], height])
    )
    feasibility = []
    for index, scenario in enumerate(scenarios):
        feasibility.append(_apply_scenario(env, index, scenario))
        env._compute_observation(index)
        env.history[index] = env.obs[index]
        env._compute_privileged(np.array([index]))

    limits = env.gains.limits()
    alive = np.ones(n, dtype=bool)
    first_fail_step = np.full(n, steps, dtype=int)
    reasons = np.full(n, "", dtype=object)
    survived_steps = np.zeros(n, dtype=int)

    height_abs_err = np.zeros(n)
    vx_abs_err = np.zeros(n)
    wz_abs_err = np.zeros(n)
    counted = np.zeros(n)
    counted_after = np.zeros(n)
    height_abs_err_after = np.zeros(n)
    vx_abs_err_after = np.zeros(n)
    tilt_values: list[list[float]] = [[] for _ in range(n)]
    saturated_steps = np.zeros(n)
    current_run = np.zeros(n)
    longest_run = np.zeros(n)
    root_start = np.array([env.data[i].qpos[0:2].copy() for i in range(n)])
    drift = np.zeros(n)
    last_height = np.array([env.data[i].qpos[2] for i in range(n)], dtype=float)

    obs, history = env.get_observations()
    for step in range(steps):
        actions = np.asarray(policy(obs, history), dtype=float)
        if actions.shape != (n, names.NUM_ACTIONS):
            raise ValueError(f"policy returned {actions.shape}, expected {(n, names.NUM_ACTIONS)}")
        obs, _priv, _reward, dones, infos, history = env.step(actions)
        stats = infos["step_stats"]

        mask = alive
        counted[mask] += 1.0
        last_height[mask] = stats["height"][mask]
        height_abs_err[mask] += np.abs(stats["height"][mask] - height)
        vx_abs_err[mask] += np.abs(stats["lin_vel_body"][mask, 0])
        wz_abs_err[mask] += np.abs(stats["ang_vel_body"][mask, 2])
        if (step + 1) * dt > settle_s:
            counted_after[mask] += 1.0
            height_abs_err_after[mask] += np.abs(stats["height"][mask] - height)
            vx_abs_err_after[mask] += np.abs(stats["lin_vel_body"][mask, 0])
        for index in np.nonzero(mask)[0]:
            tilt_values[index].append(float(stats["tilt_deg"][index]))
        saturated = np.abs(stats["torque"]) >= (limits - torque_epsilon)
        saturated_steps[mask] += saturated[mask].any(axis=1)
        current_run = np.where(saturated.any(axis=1) & mask, current_run + 1, 0.0)
        longest_run = np.maximum(longest_run, current_run)

        newly = dones & alive
        if np.any(newly):
            first_fail_step[newly] = step + 1
            reasons[newly] = [infos["termination_reason"][i] for i in np.nonzero(newly)[0]]
        survived_steps[alive] = step + 1
        drift[alive] = [
            float(np.linalg.norm(stats["root_xy"][i] - root_start[i])) for i in np.nonzero(alive)[0]
        ]
        alive = alive & ~dones

    episodes: list[EpisodeResult] = []
    for index in range(n):
        count = max(1.0, counted[index])
        count_after = max(1.0, counted_after[index])
        tilts = np.asarray(tilt_values[index]) if tilt_values[index] else np.zeros(1)
        episodes.append(
            EpisodeResult(
                seed=scenarios[index].seed,
                survived_s=float(survived_steps[index] * dt),
                survived_steps=int(survived_steps[index]),
                success=bool(survived_steps[index] >= steps),
                reason=str(reasons[index]),
                height_mae=float(height_abs_err_after[index] / count_after)
                if counted_after[index] > 0
                else float(height_abs_err[index] / count),
                height_final=float(last_height[index]),
                vx_mae=float(vx_abs_err_after[index] / count_after)
                if counted_after[index] > 0
                else float(vx_abs_err[index] / count),
                wz_mae=float(wz_abs_err[index] / count),
                tilt_p95_deg=float(np.percentile(tilts, 95)),
                tilt_max_deg=float(tilts.max()),
                torque_saturation_fraction=float(saturated_steps[index] / count),
                longest_saturation_s=float(longest_run[index] * dt),
                drift_m=float(drift[index]),
                scenario=scenarios[index],
            )
        )

    survival = np.array([episode.survived_s for episode in episodes], dtype=float)
    successes = np.array([episode.success for episode in episodes], dtype=bool)
    return _summarize(
        episodes,
        steps=steps,
        dt=dt,
        episode_length_s=episode_length_s,
        command=command,
        height=height,
        feasibility=feasibility,
        backend="cpu",
    )


def _summarize(
    episodes: Sequence[EpisodeResult],
    *,
    steps: int,
    dt: float,
    episode_length_s: float,
    command: tuple[float, float, float] | None,
    height: float,
    feasibility: Sequence[dict[str, Any]],
    backend: str,
) -> dict[str, Any]:
    """One summary definition for both backends, so the gate cannot drift."""
    survival = np.array([episode.survived_s for episode in episodes], dtype=float)
    successes = np.array([episode.success for episode in episodes], dtype=bool)
    summary: dict[str, Any] = {
        "episodes": len(episodes),
        "steps": int(steps),
        "policy_dt_s": float(dt),
        "window_s": float(steps * dt),
        "episode_limit_s": float(episode_length_s),
        "backend": backend,
        "successes": int(successes.sum()),
        "success_rate": float(successes.mean()) if episodes else float("nan"),
        "survival_min_s": float(survival.min()) if episodes else float("nan"),
        "survival_p10_s": float(np.percentile(survival, 10)) if episodes else float("nan"),
        "survival_median_s": float(np.median(survival)) if episodes else float("nan"),
        "survival_mean_s": float(survival.mean()) if episodes else float("nan"),
        "height_mae_m": float(np.nanmean([e.height_mae for e in episodes])) if episodes else float("nan"),
        "vx_mae_m_s": float(np.nanmean([e.vx_mae for e in episodes])) if episodes else float("nan"),
        "tilt_p95_deg": float(np.max([e.tilt_p95_deg for e in episodes])) if episodes else float("nan"),
        "tilt_max_deg": float(np.max([e.tilt_max_deg for e in episodes])) if episodes else float("nan"),
        "torque_saturation_fraction": float(
            np.max([e.torque_saturation_fraction for e in episodes])
        )
        if episodes
        else float("nan"),
        "drift_max_m": float(np.max([e.drift_m for e in episodes])) if episodes else float("nan"),
        "reasons": _reason_counts([episode.reason for episode in episodes if episode.reason]),
        "command": {
            "vx": 0.0 if command is None else command[0],
            "wz": 0.0 if command is None else command[1],
            "height": height,
        },
        "feasibility": list(feasibility),
    }
    summary["criteria"] = _criteria(summary, list(episodes), successes)
    summary["status"] = "passed" if summary["criteria"]["passed"] else "failed"
    summary["episodes_detail"] = [episode.as_dict() for episode in episodes]
    return summary


def _reason_counts(reasons: Sequence[str]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for reason in reasons:
        counts[reason] = counts.get(reason, 0) + 1
    return counts


def _criteria(summary: dict[str, Any], episodes: Sequence[EpisodeResult], successes: np.ndarray) -> dict[str, Any]:
    checks = {
        "success_rate": {
            "value": summary["success_rate"],
            "threshold": THRESHOLDS["success_rate_min"],
            "passed": bool(summary["success_rate"] >= THRESHOLDS["success_rate_min"]),
        },
        "vx_mae": {
            "value": summary["vx_mae_m_s"],
            "threshold": THRESHOLDS["vx_mae_max"],
            "passed": bool(summary["vx_mae_m_s"] <= THRESHOLDS["vx_mae_max"]),
        },
        "height_mae": {
            "value": summary["height_mae_m"],
            "threshold": THRESHOLDS["height_mae_max"],
            "passed": bool(summary["height_mae_m"] <= THRESHOLDS["height_mae_max"]),
        },
        "tilt_p95": {
            "value": summary["tilt_p95_deg"],
            "threshold": THRESHOLDS["tilt_p95_max_deg"],
            "passed": bool(summary["tilt_p95_deg"] <= THRESHOLDS["tilt_p95_max_deg"]),
        },
        "tilt_max": {
            "value": summary["tilt_max_deg"],
            "threshold": THRESHOLDS["tilt_max_max_deg"],
            "passed": bool(summary["tilt_max_deg"] <= THRESHOLDS["tilt_max_max_deg"]),
        },
        "torque_saturation": {
            "value": summary["torque_saturation_fraction"],
            "threshold": THRESHOLDS["torque_saturation_max"],
            "passed": bool(summary["torque_saturation_fraction"] <= THRESHOLDS["torque_saturation_max"]),
        },
    }
    return {
        "checks": checks,
        "passed": all(check["passed"] for check in checks.values()),
        "successes": int(successes.sum()) if len(successes) else 0,
    }


# --------------------------------------------------------------- policy adapters
class TorchPolicy:
    """Deterministic mean action of a trained ``ActorCriticSequence``."""

    def __init__(self, policy_module, device: str = "cpu"):
        import torch

        self.torch = torch
        self.device = torch.device(device)
        self.policy = policy_module.to(self.device).eval()

    @classmethod
    def from_checkpoint(cls, path, device: str = "cpu") -> "TorchPolicy":
        from ..learning.actor_critic import ActorCriticSequence, NetworkDims
        from ..learning.checkpoint import load_checkpoint

        payload = load_checkpoint(path, map_location=device)
        dims = NetworkDims(**payload.get("dims", {}))
        module = ActorCriticSequence(dims)
        module.load_state_dict(payload["policy"])
        return cls(module, device=device)

    def __call__(self, obs: np.ndarray, history: np.ndarray) -> np.ndarray:
        with self.torch.no_grad():
            obs_t = self.torch.as_tensor(obs, dtype=self.torch.float32, device=self.device)
            hist_t = self.torch.as_tensor(history, dtype=self.torch.float32, device=self.device)
            mean, _latent = self.policy.act_inference(obs_t, hist_t)
            return mean.cpu().numpy()


class OnnxPolicy:
    """Deterministic ONNX actor (the deployment artefact)."""

    def __init__(self, path, providers: Sequence[str] | None = None):
        import onnxruntime as ort

        self.session = ort.InferenceSession(
            str(path), providers=list(providers or ["CPUExecutionProvider"])
        )
        self.inputs = [entry.name for entry in self.session.get_inputs()]

    def __call__(self, obs: np.ndarray, history: np.ndarray) -> np.ndarray:
        feeds = {self.inputs[0]: obs.astype(np.float32)}
        if len(self.inputs) > 1:
            feeds[self.inputs[1]] = history.astype(np.float32)
        output = self.session.run(None, feeds)[0]
        return np.asarray(output, dtype=float).reshape(obs.shape[0], -1)


# --------------------------------------------------------------------- warp backend
def _run_stand_eval_warp(
    config: Config,
    policy: PolicyFn,
    *,
    steps: int,
    scenarios: Sequence[Scenario],
    command: tuple[float, float, float] | None,
    settle_s: float,
    torque_epsilon: float,
    device: str,
) -> dict[str, Any]:
    """The same acceptance window on ``WarpVecEnv``.

    The metrics are accumulated with the same rules as the CPU path, and the
    per-step state is taken from the env's terminal snapshot when an episode
    ends, so the failure that actually happened is scored - not the reset pose.
    """
    import torch

    from ..control.spawn import load_spawn_spec, resolve_spawn_height
    from ..envs.cpu_env import FiftyCpuEnv
    from ..envs.warp_env import WarpEnvOptions, WarpVecEnv
    from ..paths import assets_dir

    scenarios = list(scenarios)
    n = len(scenarios)
    dt = float(config.robot["control"]["policy_dt_s"])
    window_s = steps * dt
    episode_length_s = window_s + max(1.0, window_s * 0.05)

    probe = FiftyCpuEnv(assets_dir() / "mjcf" / "fifty_train.xml", config)
    nominal = resolve_spawn_height(probe, load_spawn_spec(config))[0] - float(
        config.spawn["spawn"]["wheel_clearance_m"]
    )
    height = nominal if command is None else float(command[2])
    vx = 0.0 if command is None else float(command[0])
    wz = 0.0 if command is None else float(command[1])

    env = WarpVecEnv(
        config,
        WarpEnvOptions(
            num_envs=n,
            device=device,
            seed=scenarios[0].seed if scenarios else 0,
            fixed_command=(vx, wz, height),
            episode_length_s=episode_length_s,
        ),
    )

    # Seed each world through a scratch MjData: the disturbance (including the
    # penetration check, which needs collision detection) is then identical to
    # the CPU path instead of being reimplemented in tensor maths.
    scratch = env.cpu
    spawn = env.spawn_row.detach().cpu().numpy().astype(np.float64)
    feasibility: list[dict[str, Any]] = []
    for index, scenario in enumerate(scenarios):
        mujoco.mj_resetData(scratch.model, scratch.data)
        scratch.data.qpos[:] = spawn
        scratch.data.qvel[:] = 0.0
        feasibility.append(
            _apply_scenario_to_data(scratch.model, scratch.data, scratch, scenario)
        )
        env.qpos[index] = torch.as_tensor(
            scratch.data.qpos, dtype=env.qpos.dtype, device=env.device
        )
        env.qvel[index] = torch.as_tensor(
            scratch.data.qvel, dtype=env.qvel.dtype, device=env.device
        )
    env._compute_observation()
    env.history.copy_(env.obs[:, None, :].expand_as(env.history))
    env._compute_privileged()

    # Without a captured graph one policy step costs ~100 ms of host time (about
    # 120 kernel launches, latency-bound), which would wipe out the speed-up:
    # 6000 steps would take ten minutes instead of seconds.
    graph_captured = False
    capture = getattr(env, "capture", None)
    if capture is not None:
        try:
            graph_captured = bool(capture())
        except Exception:  # pragma: no cover - capture is best effort here
            graph_captured = False

    if hasattr(policy, "bind") and not getattr(policy, "supports_warp_backend", False):
        raise ValueError(
            f"{type(policy).__name__} reads the simulator directly and is not supported on"
            " the warp backend; use --backend cpu"
        )
    if hasattr(policy, "reset"):
        policy.reset()

    limits = env.gains.limits()
    alive = np.ones(n, dtype=bool)
    survived_steps = np.zeros(n, dtype=int)
    reasons = np.full(n, "", dtype=object)
    height_abs_err = np.zeros(n)
    vx_abs_err = np.zeros(n)
    wz_abs_err = np.zeros(n)
    counted = np.zeros(n)
    counted_after = np.zeros(n)
    height_abs_err_after = np.zeros(n)
    vx_abs_err_after = np.zeros(n)
    tilt_values: list[list[float]] = [[] for _ in range(n)]
    saturated_steps = np.zeros(n)
    current_run = np.zeros(n)
    longest_run = np.zeros(n)
    drift = np.zeros(n)
    last_height = np.zeros(n)
    root_start = np.zeros((n, 2))
    tilt_limit = float(env.termination.max_tilt_deg)
    height_limit = float(env.termination.height_ratio) * float(env.nominal_height)

    obs_t, hist_t = env.get_observations()
    # The drift reference is the seeded (perturbed) root position, not the spawn.
    stats0 = _warp_step_stats(env, np.zeros(n, dtype=bool))
    root_start[:] = stats0["root_xy"]
    last_height[:] = stats0["height"]

    for step in range(steps):
        actions = np.asarray(
            policy(obs_t.detach().cpu().numpy(), hist_t.detach().cpu().numpy()), dtype=float
        )
        if actions.shape != (n, names.NUM_ACTIONS):
            raise ValueError(
                f"policy returned {actions.shape}, expected {(n, names.NUM_ACTIONS)}"
            )
        out = env.step(torch.as_tensor(actions, dtype=torch.float32, device=env.device))
        obs_t, _priv, _reward, done_t, _info, hist_t = out
        dones = done_t.detach().cpu().numpy().astype(bool)
        time_outs = env.time_outs.detach().cpu().numpy().astype(bool)
        stats = _warp_step_stats(env, dones)

        mask = alive
        counted[mask] += 1.0
        last_height[mask] = stats["height"][mask]
        height_abs_err[mask] += np.abs(stats["height"][mask] - height)
        vx_abs_err[mask] += np.abs(stats["lin_vel_body"][mask, 0])
        wz_abs_err[mask] += np.abs(stats["ang_vel_body"][mask, 2])
        if (step + 1) * dt > settle_s:
            counted_after[mask] += 1.0
            height_abs_err_after[mask] += np.abs(stats["height"][mask] - height)
            vx_abs_err_after[mask] += np.abs(stats["lin_vel_body"][mask, 0])
        for index in np.nonzero(mask)[0]:
            tilt_values[index].append(float(stats["tilt_deg"][index]))
        saturated = np.abs(stats["torque"]) >= (limits - torque_epsilon)
        saturated_steps[mask] += saturated[mask].any(axis=1)
        current_run = np.where(saturated.any(axis=1) & mask, current_run + 1, 0.0)
        longest_run = np.maximum(longest_run, current_run)

        newly = dones & alive
        for index in np.nonzero(newly)[0]:
            # The warp env reports dones but not a reason string; derive it from
            # the terminal state so the report stays comparable.
            if time_outs[index]:
                reasons[index] = "timeout"
            elif stats["tilt_deg"][index] >= tilt_limit:
                reasons[index] = "tilt"
            elif stats["height"][index] < height_limit:
                reasons[index] = "height"
            else:
                reasons[index] = "other"
        survived_steps[alive] = step + 1
        drift[alive] = [
            float(np.linalg.norm(stats["root_xy"][i] - root_start[i]))
            for i in np.nonzero(alive)[0]
        ]
        alive = alive & ~dones

    episodes: list[EpisodeResult] = []
    for index in range(n):
        count = max(1.0, counted[index])
        count_after = max(1.0, counted_after[index])
        tilts = np.asarray(tilt_values[index]) if tilt_values[index] else np.zeros(1)
        episodes.append(
            EpisodeResult(
                seed=scenarios[index].seed,
                survived_s=float(survived_steps[index] * dt),
                survived_steps=int(survived_steps[index]),
                success=bool(survived_steps[index] >= steps),
                reason=str(reasons[index]),
                height_mae=float(height_abs_err_after[index] / count_after)
                if counted_after[index] > 0
                else float(height_abs_err[index] / count),
                height_final=float(last_height[index]),
                vx_mae=float(vx_abs_err_after[index] / count_after)
                if counted_after[index] > 0
                else float(vx_abs_err[index] / count),
                wz_mae=float(wz_abs_err[index] / count),
                tilt_p95_deg=float(np.percentile(tilts, 95)),
                tilt_max_deg=float(tilts.max()),
                torque_saturation_fraction=float(saturated_steps[index] / count),
                longest_saturation_s=float(longest_run[index] * dt),
                drift_m=float(drift[index]),
                scenario=scenarios[index],
            )
        )

    summary = _summarize(
        episodes,
        steps=steps,
        dt=dt,
        episode_length_s=episode_length_s,
        command=command,
        height=height,
        feasibility=feasibility,
        backend="warp",
    )
    summary["reason_source"] = "derived_from_terminal_state"
    summary["cuda_graph"] = graph_captured
    summary["note"] = (
        "warp backend: reasons are derived from the terminal state because"
        " WarpVecEnv does not report a termination string"
    )
    return summary


def _warp_step_stats(env, dones: np.ndarray) -> dict[str, np.ndarray]:
    """Per-env state for the step that just finished, in the terminal snapshot."""
    import torch

    mask = torch.as_tensor(dones, dtype=torch.bool, device=env.device).unsqueeze(1)
    qpos = torch.where(mask, env.terminal_qpos, env.qpos).detach().cpu().numpy().astype(np.float64)
    qvel = torch.where(mask, env.terminal_qvel, env.qvel).detach().cpu().numpy().astype(np.float64)
    torque = (
        torch.where(mask, env.terminal_torques, env.torques).detach().cpu().numpy().astype(np.float64)
    )
    quat = qpos[:, 3:7]
    quat = quat / np.linalg.norm(quat, axis=1, keepdims=True)
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    rot = np.empty((qpos.shape[0], 3, 3))
    rot[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    rot[:, 0, 1] = 2.0 * (x * y - w * z)
    rot[:, 0, 2] = 2.0 * (x * z + w * y)
    rot[:, 1, 0] = 2.0 * (x * y + w * z)
    rot[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    rot[:, 1, 2] = 2.0 * (y * z - w * x)
    rot[:, 2, 0] = 2.0 * (x * z - w * y)
    rot[:, 2, 1] = 2.0 * (y * z + w * x)
    rot[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    lin_body = np.einsum("nji,nj->ni", rot, qvel[:, 0:3])
    return {
        "height": qpos[:, 2],
        "tilt_deg": np.degrees(np.arccos(np.clip(rot[:, 2, 2], -1.0, 1.0))),
        "lin_vel_body": lin_body,
        "ang_vel_body": qvel[:, 3:6],
        "root_xy": qpos[:, 0:2],
        "torque": torque,
    }

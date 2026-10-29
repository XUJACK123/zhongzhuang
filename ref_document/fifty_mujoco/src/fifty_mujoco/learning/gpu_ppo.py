"""Batched PPO on the GPU environment, with live progress reporting.

Three gradient paths, matching ``learning/ppo.py``:

* actor   <- clipped PPO surrogate (+ entropy)
* critic  <- value regression against GAE(lambda) returns
* encoder <- MSE of the latent against the true body velocity
  (``privileged[:, 0:3] / lin_vel_scale``), the asymmetric-training trick the
  Fudan project uses

Everything (rollout buffer, GAE, minibatch loop) stays on the device; only the
log line touches the host, once per update.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np
import torch

from ..config import Config
from ..control.mixed_pd import PdGains
from ..envs.warp_env import WarpEnvOptions, WarpVecEnv
from .actor_critic import ActorCriticSequence, NetworkDims
from .checkpoint import (
    CHECKPOINT_FORMAT,
    CheckpointManager,
    load_checkpoint,
    model_fingerprint,
    restore_rng_state,
    rng_state,
    save_checkpoint,
)


@dataclass
class GpuPpoConfig:
    num_envs: int = 1024
    num_steps: int = 24
    updates: int = 300
    epochs: int = 5
    minibatches: int = 4
    gamma: float = 0.99
    lam: float = 0.95
    clip: float = 0.2
    lr: float = 1e-3
    encoder_lr: float = 1e-3
    entropy_coef: float = 0.005
    value_coef: float = 1.0
    encoder_coef: float = 0.5
    max_grad_norm: float = 1.0
    desired_kl: float = 0.01
    init_std: float = 0.5
    #: Per-channel exploration std.  The wheel channels are multiplied by 10 rad/s
    #: in the PD, so the same std value would inject +-5 rad/s into the wheel loop
    #: and destroy the episode before anything can be learned.
    init_std_per_channel: tuple[float, ...] = (0.4, 0.4, 0.1, 0.4, 0.4, 0.1)
    #: Lowered from (0.2, 0.2, 0.05, ...): a deterministic evaluation showed both
    #: checkpoints already stand for >5 s with the mean action, so the short
    #: training episodes were caused by exploration noise, not by a missing
    #: skill.  Less noise -> longer episodes -> better credit assignment.
    min_std_per_channel: tuple[float, ...] = (0.1, 0.1, 0.02, 0.1, 0.1, 0.02)
    min_std: float = 0.3
    max_std: float = 1.0
    adaptive_lr: bool = True
    min_lr: float = 2e-5
    max_lr: float = 3e-3
    seed: int = 0
    device: str = "cuda:0"
    command_resample_updates: int = 1
    log_every: int = 1
    #: Deterministic evaluation (mean action, fixed commands, no noise).  This is
    #: the metric that actually answers "can it stand"; the training episode
    #: length only measures tolerance to exploration noise.
    eval_every: int = 50
    eval_steps: int = 300
    eval_num_envs: int = 64
    progress_path: Path | None = None
    #: Write the checkpoint (and a ``history.json``) every N updates.  A 3-hour
    #: unattended run must survive a crash, so 0 ("only at the end") is unsafe.
    #: 25 is the value the experiment baseline in progress.md §5.5 / §13.1 fixes;
    #: retention (``checkpoint_keep``) bounds the disk cost.
    save_every: int = 25
    #: Gate for logging ``best``; the *ordering* used to pick ``model_best.pt``
    #: lives in :data:`fifty_mujoco.learning.checkpoint.DEFAULT_BEST_ORDER`
    #: (alive fraction -> P10 survival -> median survival -> height -> vx).
    best_metric_key: str = "eval_t_fall_median"
    #: Number of numbered ``model_update_*.pt`` snapshots to retain.
    checkpoint_keep: int = 5
    #: Stop after this many consecutive evaluations that survive the whole eval
    #: window (0 disables).
    stop_on_survive: int = 0
    #: tqdm-style in-place progress bar (falls back to one line per update when
    #: tqdm is missing or stdout is not a tty).
    progress_bar: bool = True
    #: Optional leg PD override.  ``Kp = 20`` cannot hold the stance statically
    #: (needs 13.9 / 21.3 N.m while Kp*0.5 rad only reaches 10 N.m), so the
    #: policy has to run at |a| ~ 1.4-2.1.  Raising Kp puts the task back inside
    #: the natural action range at the cost of a stiffer, less Fudan-like loop.
    leg_kp: float | None = None
    leg_kd: float | None = None
    save_path: Path | None = None
    plot_path: Path | None = None
    #: Optional per-update hook ``fn(record: dict) -> None`` (live plotting).
    callback: Callable[[dict[str, Any]], None] | None = None
    #: Initial actor output bias.  With Kp = 20 the static stance needs roughly
    #: |a| ~ 1.4-2.4 on the leg channels, and a zero-mean Gaussian cannot find
    #: that by exploration in a useful number of updates.  This bias is only an
    #: initialisation: it does not change the control contract, the environment
    #: still uses the network output unchanged.  Measured: bias keeps the robot
    #: up for 0.54 s vs 0.07 s with a zero bias (open loop, no controller).
    hold_bias: tuple[float, ...] = (1.0, 1.2, 0.0, -1.0, -1.2, 0.0)
    action_scale_hint: tuple[float, ...] = (0.5, 0.5, 10.0, 0.5, 0.5, 10.0)


class GpuPpoTrainer:
    def __init__(
        self,
        config: Config,
        cfg: GpuPpoConfig | None = None,
        env: WarpVecEnv | None = None,
    ):
        self.config = config
        self.cfg = cfg or GpuPpoConfig()
        self.device = torch.device(self.cfg.device)
        torch.manual_seed(self.cfg.seed)
        np.random.seed(self.cfg.seed)

        self.env = env or WarpVecEnv(
            config,
            WarpEnvOptions(
                num_envs=self.cfg.num_envs,
                device=self.cfg.device,
                seed=self.cfg.seed,
                gains=self._resolve_gains(config),
            ),
        )
        self.num_envs = self.env.num_envs
        self.graph_ok = self.env.capture()

        dims = NetworkDims(num_privileged=self.env.num_privileged_obs)
        self.eval_env: WarpVecEnv | None = None
        if self.cfg.eval_every > 0:
            self.eval_env = WarpVecEnv(
                config,
                WarpEnvOptions(
                    num_envs=self.cfg.eval_num_envs,
                    device=self.cfg.device,
                    seed=self.cfg.seed + 991,
                    gains=self._resolve_gains(config),
                    fixed_command=(0.0, 0.0, 0.0),  # replaced below by nominal h
                ),
            )
            self.eval_env.commands[:] = torch.tensor(
                [0.0, 0.0, self.eval_env.nominal_height], device=self.device
            )
            self.eval_env.capture()
            self.eval_dt = self.eval_env.dt
        self.policy = ActorCriticSequence(dims).to(self.device)
        with torch.no_grad():
            self.policy.std.fill_(self.cfg.init_std)
            if (
                self.cfg.init_std_per_channel is not None
                and len(self.cfg.init_std_per_channel) == self.env.num_actions
            ):
                self.policy.std.copy_(
                    torch.as_tensor(
                        self.cfg.init_std_per_channel, dtype=torch.float32, device=self.device
                    )
                )
            self._std_floor = torch.as_tensor(
                self.cfg.min_std_per_channel, dtype=torch.float32, device=self.device
            )
            if self.cfg.hold_bias is not None and len(self.cfg.hold_bias) == self.env.num_actions:
                bias = torch.as_tensor(self.cfg.hold_bias, dtype=torch.float32, device=self.device)
                # keep the *torque* implied by the bias constant when Kp changes:
                # tau ~ Kp * action_scale * bias, so bias must scale as 1/Kp.
                configured_kp = float(config.robot["control"]["leg_kp"])
                actual_kp = float(self.env.gains.leg_kp)
                if actual_kp > 0.0:
                    scale = min(1.0, configured_kp / actual_kp)
                    bias[[0, 1, 3, 4]] = bias[[0, 1, 3, 4]] * scale
                self.hold_bias_tensor = bias
                self.policy.actor[-1].bias.copy_(bias)
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=self.cfg.lr)
        self.learning_rate = self.cfg.lr

        t, n = self.cfg.num_steps, self.num_envs
        self.obs_buf = torch.zeros((t, n, self.env.num_obs), device=self.device)
        self.hist_buf = torch.zeros((t, n, dims.num_history), device=self.device)
        self.priv_buf = torch.zeros((t, n, self.env.num_privileged_obs), device=self.device)
        self.act_buf = torch.zeros((t, n, self.env.num_actions), device=self.device)
        self.logp_buf = torch.zeros((t, n), device=self.device)
        self.val_buf = torch.zeros((t, n), device=self.device)
        self.rew_buf = torch.zeros((t, n), device=self.device)
        self.done_buf = torch.zeros((t, n), device=self.device)
        self.timeout_buf = torch.zeros((t, n), device=self.device)
        self.term_priv_buf = torch.zeros((t, n, self.env.num_privileged_obs), device=self.device)
        self.term_hist_buf = torch.zeros((t, n, dims.num_history), device=self.device)
        self.adv_buf = torch.zeros((t, n), device=self.device)
        self.ret_buf = torch.zeros((t, n), device=self.device)
        self.history: list[dict[str, Any]] = []
        self.start_time = time.perf_counter()
        self.total_env_steps = 0
        self.update_count = 0
        self.best_metric = float("-inf")
        self.best_update = 0
        self._survive_streak = 0
        self.checkpoints: CheckpointManager | None = None
        if self.cfg.save_path is not None:
            self.checkpoints = CheckpointManager(
                Path(self.cfg.save_path).parent, keep=self.cfg.checkpoint_keep
            )
            if self.checkpoints.best_metrics is not None:
                self.best_update = self.checkpoints.best_update

    # ------------------------------------------------------------- inference
    def _resolve_gains(self, config: Config) -> PdGains | None:
        """PD gains for the env; ``None`` keeps the config baseline."""
        if self.cfg.leg_kp is None and self.cfg.leg_kd is None:
            return None
        control = config.robot["control"]
        return PdGains(
            leg_kp=float(self.cfg.leg_kp if self.cfg.leg_kp is not None else control["leg_kp"]),
            leg_kd=float(self.cfg.leg_kd if self.cfg.leg_kd is not None else control["leg_kd"]),
            wheel_kp=float(control["wheel_kp"]),
            wheel_kd=float(control["wheel_kd"]),
            torque_limit_nm=config.effort_limit("rf0_Joint"),
            wheel_torque_limit_nm=config.effort_limit("r_wheel_Joint"),
            leg_position_scale=float(control["leg_position_scale"]),
            wheel_velocity_scale=float(control["wheel_velocity_scale"]),
        )

    def _dist(self, obs: torch.Tensor, hist: torch.Tensor):
        mean, latent = self.policy.act_inference(obs, hist)
        std = self.policy.std.expand_as(mean)
        return torch.distributions.Normal(mean, std.clamp_min(1e-4)), latent

    @torch.no_grad()
    def _rollout(self) -> dict[str, float]:
        env = self.env
        obs, hist = env.get_observations()
        priv = env.get_privileged_observations()
        reward_sum = 0.0
        for step in range(self.cfg.num_steps):
            dist, latent = self._dist(obs, hist)
            action = dist.sample()
            logp = dist.log_prob(action).sum(-1)
            value = self.policy.value(priv, latent.detach()).squeeze(-1)
            self.obs_buf[step] = obs
            self.hist_buf[step] = hist
            self.priv_buf[step] = priv
            self.act_buf[step] = action
            self.logp_buf[step] = logp
            self.val_buf[step] = value
            obs, priv, reward, done, infos, hist = env.step(action)
            self.rew_buf[step] = reward
            self.done_buf[step] = done.float()
            self.timeout_buf[step] = infos["time_outs"].float()
            self.term_priv_buf[step] = infos["terminal_privileged"]
            self.term_hist_buf[step] = infos["terminal_history"]
            reward_sum += float(reward.mean())
        self.total_env_steps += self.cfg.num_steps * self.num_envs
        return {"reward_mean": reward_sum / self.cfg.num_steps}

    @torch.no_grad()
    def _gae(self) -> None:
        env = self.env
        obs, hist = env.get_observations()
        priv = env.get_privileged_observations()
        latent = self.policy.encode(hist)
        last_value = self.policy.value(priv, latent.detach()).squeeze(-1)
        # one batched forward for every step's terminal state instead of T of them
        t, n = self.cfg.num_steps, self.num_envs
        term_latent = self.policy.encode(self.term_hist_buf.reshape(t * n, -1))
        term_value = self.policy.value(
            self.term_priv_buf.reshape(t * n, -1), term_latent.detach()
        ).squeeze(-1).reshape(t, n)
        advantage = torch.zeros(n, device=self.device)
        for step in reversed(range(t)):
            done = self.done_buf[step]
            timeout = self.timeout_buf[step]
            next_value = last_value if step == t - 1 else self.val_buf[step + 1]
            # failure -> no bootstrap; timeout -> bootstrap from the terminal state
            bootstrap = torch.where(timeout > 0.5, term_value[step], torch.zeros(n, device=self.device))
            alive_next = next_value * (1.0 - done) + bootstrap * done
            delta = self.rew_buf[step] + self.cfg.gamma * alive_next - self.val_buf[step]
            advantage = delta + self.cfg.gamma * self.cfg.lam * (1.0 - done) * advantage
            self.adv_buf[step] = advantage
        self.ret_buf.copy_(self.adv_buf + self.val_buf)
        flat = self.adv_buf.reshape(-1)
        self.adv_buf.copy_(((flat - flat.mean()) / (flat.std() + 1e-8)).reshape_as(self.adv_buf))

    # ---------------------------------------------------------------- update
    def _update(self) -> dict[str, float]:
        t, n = self.cfg.num_steps, self.num_envs
        obs = self.obs_buf.reshape(t * n, -1)
        hist = self.hist_buf.reshape(t * n, -1)
        priv = self.priv_buf.reshape(t * n, -1)
        act = self.act_buf.reshape(t * n, -1)
        logp_old = self.logp_buf.reshape(-1)
        adv = self.adv_buf.reshape(-1)
        ret = self.ret_buf.reshape(-1)
        target_velocity = priv[:, 0:3] * 0.5

        batch = t * n
        size = batch // self.cfg.minibatches
        keys = ("policy_loss", "value_loss", "encoder_loss", "kl", "entropy", "clip_fraction")
        acc = torch.zeros(len(keys), device=self.device)
        times = 0
        for _ in range(self.cfg.epochs):
            perm = torch.randperm(batch, device=self.device)
            for start in range(0, batch - size + 1, size):
                idx = perm[start : start + size]
                mean, latent = self.policy.act_inference(obs[idx], hist[idx])
                dist = torch.distributions.Normal(mean, self.policy.std.clamp_min(1e-4).expand_as(mean))
                logp = dist.log_prob(act[idx]).sum(-1)
                entropy = dist.entropy().sum(-1).mean()
                ratio = torch.exp(logp - logp_old[idx])
                unclipped = ratio * adv[idx]
                clipped = ratio.clamp(1.0 - self.cfg.clip, 1.0 + self.cfg.clip) * adv[idx]
                policy_loss = -torch.min(unclipped, clipped).mean()
                value = self.policy.value(priv[idx], latent.detach()).squeeze(-1)
                value_loss = 0.5 * (value - ret[idx]).pow(2).mean()
                encoder_loss = (latent - target_velocity[idx]).pow(2).mean()
                loss = (
                    policy_loss
                    + self.cfg.value_coef * value_loss
                    + self.cfg.encoder_coef * encoder_loss
                    - self.cfg.entropy_coef * entropy
                )
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.cfg.max_grad_norm)
                self.optimizer.step()

                with torch.no_grad():
                    approx_kl = ((ratio - 1.0) - (logp - logp_old[idx])).mean()
                    clip_frac = ((ratio - 1.0).abs() > self.cfg.clip).float().mean()
                    # accumulate on the device; one host read per update
                    acc += torch.stack(
                        [
                            policy_loss.detach(),
                            value_loss.detach(),
                            encoder_loss.detach(),
                            approx_kl.detach(),
                            entropy.detach(),
                            clip_frac.detach(),
                        ]
                    )
                times += 1
        stats = dict(zip(keys, (acc / max(1, times)).cpu().tolist()))
        with torch.no_grad():
            # keep exploration alive: a collapsed std cannot recover
            self.policy.std.clamp_(
                self._std_floor, torch.full_like(self._std_floor, self.cfg.max_std)
            )
            if self.cfg.adaptive_lr:
                if stats["kl"] > 2.0 * self.cfg.desired_kl:
                    self.learning_rate = max(self.cfg.min_lr, self.learning_rate / 1.5)
                elif stats["kl"] < 0.5 * self.cfg.desired_kl:
                    self.learning_rate = min(self.cfg.max_lr, self.learning_rate * 1.5)
                for group in self.optimizer.param_groups:
                    group["lr"] = self.learning_rate
            explained = 1.0 - (self.ret_buf - self.val_buf).var() / (self.ret_buf.var() + 1e-8)
        stats["explained_variance"] = float(explained)
        stats["std_mean"] = float(self.policy.std.detach().mean())
        stats["lr"] = float(self.learning_rate)
        return stats

    # ------------------------------------------------------------------ train
    def evaluate(self, steps: int | None = None) -> dict[str, float]:
        """Deterministic rollout: mean action, no exploration noise.

        This is the *training* signal.  Acceptance uses
        :func:`fifty_mujoco.evaluation.stand_eval.run_stand_eval`, which keeps
        per-episode first-failure statistics on the CPU model.  Samples taken
        after an automatic reset are excluded here as well: the spawn pose must
        never be averaged in as if the robot were still standing.
        """
        env = self.eval_env
        if env is None:
            return {}
        steps = self.cfg.eval_steps if steps is None else steps
        n = env.num_envs
        env.reset(resample_commands=False)
        env.commands[:] = torch.tensor(
            [0.0, 0.0, env.nominal_height], device=self.device
        )
        env._refresh_command_channels()
        env.drain_stats()  # clear the counters, we want only this window
        first_fall = torch.full((n,), float(steps), device=self.device)
        alive = torch.ones(n, dtype=torch.bool, device=self.device)
        height_err = torch.zeros(n, device=self.device)
        vx_err = torch.zeros(n, device=self.device)
        tilt_max = torch.zeros(n, device=self.device)
        counts = torch.zeros(n, device=self.device)
        for k in range(steps):
            with torch.no_grad():
                mean, _ = self.policy.act_inference(
                    env.obs, env.history.reshape(n, -1)
                )
            _obs, _priv, _rew, done, _infos, _hist = env.step(mean)
            newly = done & alive
            first_fall = torch.where(
                newly, torch.full_like(first_fall, float(k + 1)), first_fall
            )
            # keep only the samples that still belong to the running episode
            mask = alive & ~done
            rotation = env._quat_to_mat(env.qpos[:, 3:7])
            lin_body = torch.einsum(
                "bij,bj->bi", rotation.transpose(1, 2), env.qvel[:, 0:3]
            )
            height_err += torch.where(mask, (env.qpos[:, 2] - env.commands[:, 2]).abs(), torch.zeros_like(height_err))
            vx_err += torch.where(mask, (env.commands[:, 0] - lin_body[:, 0]).abs(), torch.zeros_like(vx_err))
            tilt = torch.rad2deg(torch.acos(rotation[:, 2, 2].clamp(-1.0, 1.0)))
            tilt_max = torch.maximum(tilt_max, torch.where(mask, tilt, torch.zeros_like(tilt)))
            counts += mask.to(torch.float32)
            alive = alive & ~done
        episodes = env.drain_stats()
        seconds = first_fall * env.dt
        count = counts.clamp_min(1.0)
        return {
            # mean episode length observed inside the window: the honest
            # "how long does it stay up" number
            "eval_episode_len": float(episodes["length_mean"]),
            "eval_t_fall_median": float(seconds.median()),
            "eval_t_fall_p10": float(torch.quantile(seconds, 0.1)),
            "eval_t_fall_min": float(seconds.min()),
            "eval_t_fall_mean": float(seconds.mean()),
            "eval_survival_s": float(seconds.mean()),
            "eval_alive_frac": float(alive.to(torch.float32).mean()),
            "eval_height_mae": float((height_err / count).mean()),
            "eval_vx_mae": float((vx_err / count).mean()),
            "eval_tilt_max_deg": float(tilt_max.max()),
            "eval_steps": steps,
        }

    def _progress(self, record: dict[str, Any], total_updates: int) -> str:
        """One compact line per update, printed and appended to the log file."""
        elapsed = record["elapsed_s"]
        eta = 0.0
        budget = getattr(self, "_budget_s", None)
        if budget is not None:
            eta = max(0.0, budget - elapsed)
        elif record["update"] > 1:
            eta = elapsed / record["update"] * max(0, total_updates - record["update"])
        eval_txt = ""
        if "eval_t_fall_median" in record:
            eval_txt = (
                f" | eval ep {record['eval_episode_len']:4.2f}s"
                f" fall {record['eval_t_fall_median']:4.2f}s"
                f" p10 {record['eval_t_fall_p10']:4.2f}s"
                f" h±{record['eval_height_mae']:.3f}"
                f" tilt {record['eval_tilt_max_deg']:4.1f}"
                f" vx±{record['eval_vx_mae']:.2f}"
            )
        return (
            f"[{record['update']:4d}] {record['elapsed_s']/60:5.1f}min eta {eta/60:4.1f}min "
            f"len {record['length_mean']:5.2f}s ret {record['return_mean']:+7.2f} "
            f"rew {record['reward_mean']:+.4f} ent {record['entropy']:+.2f} "
            f"ev {record['explained_variance']:+.2f} "
            f"| tilt {record['tilt']:4d} h {record['height']:4d} c {record['illegal_contact']:4d} "
            f"to {record['time_out']:3d} | {record['sps']/1e3:5.1f}k sps{eval_txt}"
        )

    def _make_bar(self, deadline: float | None):
        """tqdm progress bar over the *time budget* (the run is time bounded)."""
        if not self.cfg.progress_bar or deadline is None:
            return None
        try:
            from tqdm import tqdm
        except ImportError:  # pragma: no cover - optional dependency
            return None
        total = max(1.0, deadline - time.perf_counter())
        self._budget_s = total
        return tqdm(
            total=total,
            unit="s",
            desc="train",
            ncols=110,
            leave=True,
            bar_format=(
                "{desc}: {percentage:3.0f}%|{bar}| {n:.0f}/{total:.0f}s "
                "[{elapsed}<{remaining}] {postfix}"
            ),
        )

    def _bar_update(self, bar, record: dict[str, Any]) -> None:
        bar.n = min(record["elapsed_s"], float(bar.total or record["elapsed_s"]))
        length = record["length_mean"]
        ret = record["return_mean"]
        postfix = (
            f"up {record['update']} | {record['sps']/1e3:.0f}k sps | "
            f"len {length:.2f}s | ret {ret:+.2f}"
            if length == length  # not NaN
            else f"up {record['update']} | {record['sps']/1e3:.0f}k sps | warming up"
        )
        if "eval_t_fall_median" in record:
            postfix += (
                f" | eval ep {record['eval_episode_len']:.2f}s"
                f" fall {record['eval_t_fall_median']:.2f}s"
            )
        bar.set_postfix_str(postfix)
        bar.refresh()

    def _bar_line(self, record: dict[str, Any], width: int = 28) -> str:
        """tqdm-shaped progress line that also works in a redirected log."""
        budget = getattr(self, "_budget_s", None) or 1.0
        elapsed = record["elapsed_s"]
        frac = min(1.0, elapsed / budget)
        filled = int(round(width * frac))
        bar = "\u2588" * filled + "\u2591" * (width - filled)
        eta = max(0.0, budget - elapsed)
        postfix = (
            f"up {record['update']} | {record['sps']/1e3:.0f}k sps"
        )
        length = record["length_mean"]
        if length == length:
            postfix += f" | len {length:.2f}s | ret {record['return_mean']:+.2f}"
        if "eval_t_fall_median" in record:
            postfix += (
                f" | eval ep {record['eval_episode_len']:.2f}s"
                f" fall {record['eval_t_fall_median']:.2f}s"
            )
        return (
            f"train {frac*100:3.0f}%|{bar}| "
            f"{elapsed:.0f}/{budget:.0f}s [{elapsed/60:.1f}<{eta/60:.1f}min] {postfix}"
        )

    def train(self, verbose: bool = True, deadline: float | None = None) -> dict[str, Any]:
        stop_reason = "max_updates"
        bar = self._make_bar(deadline) if verbose else None
        for update in range(1, self.cfg.updates + 1):
            if self.cfg.command_resample_updates and update % self.cfg.command_resample_updates == 0:
                self.env.resample_commands()
            rollout = self._rollout()
            self._gae()
            stats = self._update()
            episodes = self.env.drain_stats()
            eval_metrics: dict[str, float] = {}
            if self.eval_env is not None and update % max(1, self.cfg.eval_every) == 0:
                eval_metrics = self.evaluate()
            elapsed = time.perf_counter() - self.start_time
            record = {
                "update": update,
                "reward_mean": rollout["reward_mean"],
                "return_mean": episodes["return_mean"],
                "length_mean": episodes["length_mean"],
                "episodes": episodes["episodes"],
                "tilt": episodes["tilt"],
                "height": episodes["height"],
                "illegal_contact": episodes["illegal_contact"],
                "time_out": episodes["time_out"],
                "sps": self.total_env_steps / max(elapsed, 1e-9),
                "elapsed_s": elapsed,
                "env_steps": self.total_env_steps,
                **stats,
                **eval_metrics,
            }
            self.history.append(record)
            self.update_count = update
            metric = eval_metrics.get(self.cfg.best_metric_key)
            if metric is not None and metric > self.best_metric:
                self.best_metric = float(metric)
                self.best_update = update
                if self.checkpoints is not None and eval_metrics:
                    self.checkpoints.maybe_save_best(update, self._checkpoint_payload(), eval_metrics)
            record["best_metric"] = self.best_metric
            record["best_update"] = self.best_update
            if verbose:
                line = self._progress(record, self.cfg.updates)
                if bar is not None:
                    self._bar_update(bar, record)
                    if self.cfg.log_every and update % self.cfg.log_every == 0:
                        bar.write(line)
                elif self.cfg.log_every and update % self.cfg.log_every == 0:
                    # stdout is redirected: emit a tqdm-shaped line so the
                    # progress is still visible in the log
                    print(self._bar_line(record), flush=True)
            if self.cfg.progress_path is not None:
                with open(self.cfg.progress_path, "a", encoding="utf-8") as handle:
                    handle.write(self._progress(record, self.cfg.updates) + "\n")
            if (
                self.cfg.save_every
                and update % self.cfg.save_every == 0
                and self.cfg.save_path is not None
            ):
                self.save(Path(self.cfg.save_path), numbered=True)
                with open(
                    Path(self.cfg.save_path).with_name("history.json"), "w", encoding="utf-8"
                ) as handle:
                    json.dump(self.history, handle)
            if self.cfg.callback is not None:
                self.cfg.callback(record)
            if self.cfg.stop_on_survive and "eval_t_fall_median" in record:
                window = record["eval_steps"] * self.eval_dt * 0.99
                if record["eval_t_fall_median"] >= window:
                    self._survive_streak += 1
                else:
                    self._survive_streak = 0
                if self._survive_streak >= self.cfg.stop_on_survive:
                    stop_reason = "solved"
                    break
            if deadline is not None and time.perf_counter() >= deadline:
                stop_reason = "deadline"
                break
        if bar is not None:
            bar.n = bar.total
            bar.refresh()
            bar.close()
        if self.cfg.save_path is not None:
            self.save(Path(self.cfg.save_path))
        return {
            "updates": len(self.history),
            "stop_reason": stop_reason,
            "env_steps": self.total_env_steps,
            "elapsed_s": time.perf_counter() - self.start_time,
            "history": self.history,
            "final": self.history[-1] if self.history else {},
            "graph": self.graph_ok,
            "capture_error": getattr(self.env, "capture_error", None),
            "num_envs": self.num_envs,
        }

    # -------------------------------------------------------------- utilities
    def _checkpoint_payload(self) -> dict[str, Any]:
        """Everything needed to resume *or* to trace what produced the weights."""
        cfg_dict = {
            key: (str(value) if isinstance(value, Path) else value)
            for key, value in vars(self.cfg).items()
        }
        cfg_dict["callback"] = None
        return {
            "format": CHECKPOINT_FORMAT,
            "policy": self.policy.state_dict(),
            "optimizer": self.optimizer.state_dict(),
            "dims": vars(self.policy.dims),
            "cfg": cfg_dict,
            "history": self.history,
            "env_steps": self.total_env_steps,
            "update": self.update_count,
            "stage": "gpu-ppo",
            "learning_rate": self.learning_rate,
            "rng": rng_state(),
            "fingerprint": model_fingerprint(),
            "eval_version": "stand_eval/v1",
            # everything the deployment side needs next to the weights
            "pd_gains": {
                "leg_kp": self.env.gains.leg_kp,
                "leg_kd": self.env.gains.leg_kd,
                "wheel_kd": self.env.gains.wheel_kd,
                "leg_torque_limit_nm": self.env.gains.torque_limit_nm,
                "wheel_torque_limit_nm": self.env.gains.wheel_torque_limit_nm,
                "leg_position_scale": self.env.gains.leg_position_scale,
                "wheel_velocity_scale": self.env.gains.wheel_velocity_scale,
            },
            "spawn_z_m": self.env.spawn_z,
            "best_update": self.best_update,
        }

    def save(self, path: Path, numbered: bool = False) -> None:
        """Atomic save; ``numbered`` also writes an immutable snapshot."""
        payload = self._checkpoint_payload()
        if numbered and self.checkpoints is not None:
            self.checkpoints.save_numbered(self.update_count, payload)
        save_checkpoint(path, payload)
        if self.checkpoints is not None:
            self.checkpoints.save_last(self.update_count, payload)

    def load(self, path: Path, resume: bool = True) -> dict[str, Any]:
        """Load a checkpoint.

        ``resume=True`` restores the optimiser, learning rate, update counter,
        history and RNG state (same experiment).  ``resume=False`` only loads
        the weights (a new experiment initialised from these weights): reusing
        the old optimiser state and the old evaluation scores would be wrong.
        """
        payload = load_checkpoint(path, map_location=self.device)
        self.policy.load_state_dict(payload["policy"])
        if not resume:
            return {
                "resumed": False,
                "update": 0,
                "fingerprint_match": None,
            }
        if payload.get("optimizer") is not None:
            self.optimizer.load_state_dict(payload["optimizer"])
        self.learning_rate = float(payload.get("learning_rate", self.cfg.lr))
        for group in self.optimizer.param_groups:
            group["lr"] = self.learning_rate
        self.history = payload.get("history", [])
        self.total_env_steps = int(payload.get("env_steps", 0))
        self.update_count = int(payload.get("update", 0))
        self.best_update = int(payload.get("best_update", 0))
        restore_rng_state(payload.get("rng"))
        fingerprint = payload.get("fingerprint")
        match = None
        if fingerprint is not None:
            current = model_fingerprint()
            match = fingerprint == current
        return {
            "resumed": True,
            "update": self.update_count,
            "env_steps": self.total_env_steps,
            "fingerprint_match": match,
        }


def plot_progress(history: list[dict[str, Any]], path: Path, title: str = "") -> Path:
    """Six-panel training progress figure (Chinese labels, no seaborn)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    # Chinese labels: pick any installed CJK family, else fall back to English.
    cjk = None
    for candidate in ("Noto Sans CJK SC", "Noto Sans CJK TC", "Noto Sans CJK JP",
                      "WenQuanYi Zen Hei", "Source Han Sans SC", "SimHei"):
        try:
            font_manager.findfont(candidate, fallback_to_default=False)
            cjk = candidate
            break
        except Exception:
            continue
    labels_cn = cjk is not None
    if cjk:
        matplotlib.rcParams["font.sans-serif"] = [cjk, "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

    def t(cn: str, en: str) -> str:
        return cn if labels_cn else en

    updates = [r["update"] for r in history]
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.5))
    ax = axes[0, 0]
    ax.plot(updates, [r["reward_mean"] for r in history], color="#1f77b4")
    ax.set_title(t("每步平均奖励", "mean reward per step"))
    ax.set_xlabel(t("更新次数", "update"))
    ax.grid(alpha=0.3)

    ax = axes[0, 1]
    ax.plot(updates, [r["return_mean"] for r in history], color="#d62728")
    ax.set_title(t("回合回报", "episode return"))
    ax.set_xlabel(t("更新次数", "update"))
    ax.grid(alpha=0.3)

    ax = axes[0, 2]
    ax.plot(updates, [r["length_mean"] for r in history], color="#2ca02c")
    ax.axhline(20.0, ls="--", lw=1, color="gray")
    ax.set_title(t("回合存活时长 (s)", "episode length (s)"))
    ax.set_xlabel(t("更新次数", "update"))
    ax.grid(alpha=0.3)

    ax = axes[1, 0]
    ax.plot(updates, [r["kl"] for r in history], color="#9467bd")
    ax.set_title(t("近似 KL", "approx KL"))
    ax.set_xlabel(t("更新次数", "update"))
    ax.grid(alpha=0.3)

    ax = axes[1, 1]
    ax.plot(updates, [r["entropy"] for r in history], label=t("策略熵", "entropy"), color="#ff7f0e")
    ax.plot(updates, [r["explained_variance"] for r in history], label=t("价值解释方差", "explained var"), color="#17becf")
    ax.legend(fontsize=8)
    ax.set_title(t("熵 / 解释方差", "entropy / explained var"))
    ax.set_xlabel(t("更新次数", "update"))
    ax.grid(alpha=0.3)

    ax = axes[1, 2]
    ax.plot(updates, [r["tilt"] for r in history], label=t("倾倒", "tilt"), alpha=0.8)
    ax.plot(updates, [r["height"] for r in history], label=t("高度过低", "low height"), alpha=0.8)
    ax.plot(updates, [r["illegal_contact"] for r in history], label=t("非法接触", "illegal contact"), alpha=0.8)
    ax.plot(updates, [r["time_out"] for r in history], label=t("超时", "time-out"), alpha=0.8)
    ax.legend(fontsize=8)
    ax.set_title(t("终止原因计数", "termination counts"))
    ax.set_xlabel(t("更新次数", "update"))
    ax.grid(alpha=0.3)

    fig.suptitle(title or t("GPU 训练进度", "GPU training progress"))
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path

"""PPO runner with the training-stage state machine (doc sections 7.3 - 7.6)."""

from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import torch

from ..config import Config
from ..envs.fifty_env import EnvOptions, FiftyVecEnv
from .actor_critic import ActorCriticSequence, NetworkDims
from .ppo import Ppo, PpoConfig
from .storage import RolloutStorage

STAGES = (
    "ASSET_VERIFIED",
    "SPAWN_VERIFIED",
    "GAS_SPRING_VERIFIED",
    "CONTRACT_VERIFIED",
    "TEACHER_RESTORED",
    "CRITIC_WARMUP",
    "TEACHER_ASSISTED",
    "ZERO_TEACHER_TRIAL",
    "INDEPENDENT_PPO",
)


@dataclass
class RunnerConfig:
    num_envs: int = 64
    num_steps_per_env: int = 48
    max_updates: int = 1000
    max_wall_time_s: float | None = 10800.0
    save_interval: int = 50
    critic_warmup_updates: int = 20
    teacher_schedule: tuple[float, ...] = (0.1, 0.05, 0.025, 0.01, 0.0)
    stage_updates_per_level: int = 30
    seed: int = 0
    device: str = "cpu"
    log_dir: str | None = None


class OnPolicyRunner:
    def __init__(
        self,
        config: Config,
        env_options: EnvOptions | None = None,
        runner_config: RunnerConfig | None = None,
        ppo_config: PpoConfig | None = None,
        checkpoint: str | Path | None = None,
    ):
        self.config = config
        self.runner_cfg = runner_config or RunnerConfig()
        self.ppo_cfg = ppo_config or PpoConfig()
        self.env_options = env_options or EnvOptions()
        self.env_options.num_envs = self.runner_cfg.num_envs
        self.env_options.seed = self.runner_cfg.seed
        self.env = FiftyVecEnv(config, self.env_options)

        dims = NetworkDims(
            num_obs=self.env.num_obs,
            num_history=self.env.obs_history_len * self.env.num_obs,
            latent_dim=3,
            num_actions=self.env.num_actions,
            num_privileged=self.env.num_privileged_obs,
        )
        self.policy = ActorCriticSequence(dims).to(self.runner_cfg.device)
        self.ppo = Ppo(self.policy, self.ppo_cfg)
        self.storage = RolloutStorage(
            self.runner_cfg.num_steps_per_env,
            self.env.num_envs,
            self.env.num_obs,
            self.env.num_privileged_obs,
            self.env.obs_history_len * self.env.num_obs,
            self.env.num_actions,
            device=self.runner_cfg.device,
        )

        self.update_count = 0
        self.stage_index = 5  # CRITIC_WARMUP
        self.teacher_coef = self.runner_cfg.teacher_schedule[0]
        self.teacher = None
        self.teacher_std = None
        self._level_start = 0
        self.start_time = 0.0
        self.elapsed_updates = 0
        self.log_dir = Path(self.runner_cfg.log_dir) if self.runner_cfg.log_dir else None
        self.history_log: list[dict[str, Any]] = []
        self.eval_log: list[dict[str, Any]] = []

        if checkpoint:
            self.load(checkpoint)

    # ------------------------------------------------------------------ stages
    @property
    def stage(self) -> str:
        return STAGES[self.stage_index]

    def _apply_stage(self) -> None:
        stage = self.stage
        if stage == "CRITIC_WARMUP":
            self.ppo.config.policy_loss_scale = 0.0
            self.ppo.config.encoder_loss_scale = 0.0
            self.ppo.config.entropy_coef = 0.0
            self.ppo.set_teacher(None, None, 0.0)
        elif stage == "TEACHER_ASSISTED":
            self.ppo.config.policy_loss_scale = 1.0
            self.ppo.config.encoder_loss_scale = 1.0
            self.ppo.config.entropy_coef = self.ppo_cfg.entropy_coef
            self.ppo.set_teacher(self.teacher, self.teacher_std, self.teacher_coef)
        else:  # ZERO_TEACHER_TRIAL / INDEPENDENT_PPO
            self.ppo.config.policy_loss_scale = 1.0
            self.ppo.config.encoder_loss_scale = 1.0
            self.ppo.config.entropy_coef = self.ppo_cfg.entropy_coef
            self.ppo.set_teacher(None, None, 0.0)

    def attach_teacher(self, teacher_path: str | Path) -> dict[str, Any]:
        from ..transfer.teacher import restore_teacher

        teacher = restore_teacher(teacher_path, self.policy.dims, device=self.runner_cfg.device)
        self.teacher = teacher.policy
        self.teacher_std = teacher.std
        self.teacher_report = teacher.report
        return teacher.report

    # ------------------------------------------------------------------- train
    def train(self, verbose: bool = True) -> dict[str, Any]:
        storage = self.storage
        self.start_time = time.time()
        stop_reason = "max_updates"

        while True:
            if self.runner_cfg.max_wall_time_s is not None:
                if time.time() - self.start_time >= self.runner_cfg.max_wall_time_s:
                    stop_reason = "wall_time"
                    break
            if self.update_count - self.elapsed_updates >= self.runner_cfg.max_updates:
                stop_reason = "max_updates"
                break
            if self.stage_index == 5 and self.update_count >= self.runner_cfg.critic_warmup_updates:
                self.stage_index = 6 if self.teacher is not None else 8
                self._level_start = self.update_count
            elif self.stage_index == 6 and self._stage_progress() >= self.runner_cfg.stage_updates_per_level:
                self.teacher_coef = self._next_teacher_coef()
                self._level_start = self.update_count
                if self.teacher_coef <= 0.0:
                    self.stage_index = 8
            self._apply_stage()

            rollout = self._collect_rollout()
            stats = self.ppo.update(storage)
            self.update_count += 1
            stats.update(
                {
                    "update": self.update_count,
                    "stage": self.stage,
                    "teacher_coef": float(self.ppo.teacher_coef),
                    "reward_mean": float(np.mean(rollout["rewards"])),
                    "episode_return_mean": float(np.mean(rollout["episode_returns"])),
                    "done_fraction": float(np.mean(rollout["dones"])),
                }
            )
            self.history_log.append(stats)
            if verbose and (self.update_count % max(1, self.runner_cfg.save_interval // 10) == 0):
                print(
                    f"[{self.update_count:5d}] {stats['stage']:<18} "
                    f"rew {stats['reward_mean']:+.4f} "
                    f"ret {stats['episode_return_mean']:+.3f} "
                    f"pi {stats['policy_loss']:+.4f} v {stats['value_loss']:.4f} "
                    f"kl {stats['kl']:.5f} tau_T {stats['teacher_coef']:.3f}"
                )
            if self.log_dir and self.update_count % self.runner_cfg.save_interval == 0:
                self.save(self.log_dir / f"model_{self.update_count}.pt")

        if self.log_dir:
            self.save(self.log_dir / "model_last.pt")
        return {
            "updates": self.update_count,
            "stop_reason": stop_reason,
            "final_stage": self.stage,
            "history": self.history_log,
            "elapsed_s": time.time() - self.start_time,
        }

    def _stage_progress(self) -> int:
        return self.update_count - self._level_start

    def _next_teacher_coef(self) -> float:
        schedule = self.runner_cfg.teacher_schedule
        if self.teacher_coef in schedule:
            index = schedule.index(self.teacher_coef)
            return schedule[min(index + 1, len(schedule) - 1)]
        return 0.0

    def _collect_rollout(self) -> dict[str, np.ndarray]:
        storage = self.storage
        storage.clear()
        env = self.env
        rewards_log = []
        dones_log = []
        returns_log = []
        obs, history = env.get_observations()
        for _ in range(self.runner_cfg.num_steps_per_env):
            privileged = env.get_privileged_observations()
            obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32, device=self.runner_cfg.device)
            hist_t = torch.as_tensor(np.asarray(history), dtype=torch.float32, device=self.runner_cfg.device)
            priv_t = torch.as_tensor(np.asarray(privileged), dtype=torch.float32, device=self.runner_cfg.device)
            actions, log_prob, values, mu, sigma = self.ppo.act(obs_t, hist_t, priv_t)

            next_obs, next_priv, reward, done, infos, next_history = env.step(actions.cpu().numpy())

            terminal_privileged = np.zeros_like(privileged)
            if "terminal_obs" in infos:
                ids = infos["reset_env_ids"]
                terminal_privileged[ids] = infos["terminal_obs"]
            storage.add(
                obs_t,
                hist_t,
                priv_t,
                actions,
                log_prob,
                values,
                torch.as_tensor(np.asarray(reward), dtype=torch.float32, device=self.runner_cfg.device),
                torch.as_tensor(done.astype(np.float32), device=self.runner_cfg.device),
                mu,
                sigma,
                terminal_privileged=torch.as_tensor(np.asarray(terminal_privileged), dtype=torch.float32, device=self.runner_cfg.device),
                timeouts=torch.as_tensor(infos["time_outs"].astype(np.float32), device=self.runner_cfg.device),
            )
            rewards_log.append(reward)
            dones_log.append(done)
            returns_log.append(infos["episode_returns"])
            obs, history = next_obs, next_history

        privileged = env.get_privileged_observations()
        last_values = self.ppo.value_of(
            torch.as_tensor(np.asarray(privileged), dtype=torch.float32, device=self.runner_cfg.device),
            torch.as_tensor(np.asarray(history), dtype=torch.float32, device=self.runner_cfg.device),
        )
        terminal_values = self.ppo.value_of(
            storage.terminal_privileged.reshape(-1, self.env.num_privileged_obs),
            storage.history.reshape(storage.num_steps * storage.num_envs, -1),
        ).reshape(storage.num_steps, storage.num_envs)
        storage.compute_returns(last_values, self.ppo_cfg.gamma, self.ppo_cfg.lam, terminal_values)
        return {
            "rewards": np.asarray(rewards_log),
            "dones": np.asarray(dones_log),
            "episode_returns": np.asarray(returns_log),
        }

    # --------------------------------------------------------------------- i/o
    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "ppo": self.ppo.state_dict(),
            "update_count": self.update_count,
            "stage": self.stage,
            "teacher_coef": self.teacher_coef,
            "dims": self.policy.dims.__dict__,
            "runner": self.runner_cfg.__dict__,
            "ppo_config": self.ppo_cfg.__dict__,
            "env": {
                "num_privileged_obs": self.env.num_privileged_obs,
                "critic_input_dim": self.env.critic_input_dim,
                "num_obs": self.env.num_obs,
                "num_actions": self.env.num_actions,
                "spring_count": self.env.spring_count,
            },
            "model_hashes": {
                "xml": self.env.single.model_summary()["xml_sha256"],
                "policy_contract": _hash(self.config.workspace_path("ref_document/fifty_mujoco/configs/policy_contract.yaml"))
                if self.config.workspace_path("ref_document/fifty_mujoco/configs/policy_contract.yaml").is_file()
                else None,
            },
            "torch_rng": torch.get_rng_state(),
            "numpy_rng": np.random.get_state(),
            "history": self.history_log[-20:],
        }
        torch.save(payload, path)
        return path

    def load(self, path: str | Path) -> dict[str, Any]:
        payload = torch.load(Path(path), map_location=self.runner_cfg.device, weights_only=False)
        saved = payload.get("env", {})
        expected = {
            "num_privileged_obs": self.env.num_privileged_obs,
            "num_obs": self.env.num_obs,
            "spring_count": self.env.spring_count,
        }
        for key, value in expected.items():
            if key in saved and saved[key] != value:
                raise ValueError(
                    f"checkpoint schema mismatch for {key}: saved {saved[key]}, env {value}. "
                    "Changing the spring count or observation layout requires a new experiment."
                )
        self.ppo.load_state_dict(payload["ppo"])
        self.update_count = int(payload.get("update_count", 0))
        self.stage_index = STAGES.index(payload["stage"]) if payload.get("stage") in STAGES else 5
        self.teacher_coef = float(payload.get("teacher_coef", 0.0))
        self._level_start = self.update_count
        if "torch_rng" in payload:
            torch.set_rng_state(payload["torch_rng"])
        self.elapsed_updates = 0
        return payload


def _hash(path: Path) -> str | None:
    if not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()

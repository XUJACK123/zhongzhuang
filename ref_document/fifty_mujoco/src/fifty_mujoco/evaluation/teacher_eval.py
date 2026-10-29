"""P6: run the restored teacher on the target model (transfer baseline)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import torch

from ..config import Config
from ..envs.fifty_env import EnvOptions, FiftyVecEnv
from ..learning.actor_critic import NetworkDims
from ..transfer.teacher import restore_teacher

EVAL_COMMANDS: dict[str, tuple[float, float]] = {
    "stand": (0.0, 0.0),
    "forward": (0.3, 0.0),
    "backward": (-0.3, 0.0),
    "turn_left": (0.0, 0.3),
    "turn_right": (0.0, -0.3),
}


def _episode(config: Config, teacher, vx: float, wz: float, seconds: float, seed: int) -> dict[str, Any]:
    env = FiftyVecEnv(
        config,
        EnvOptions(num_envs=1, seed=seed, fixed_command=(vx, wz, 0.464751), episode_length_s=20.0),
    )
    env.commands.values[:, 2] = env.nominal_height
    obs, history = env.get_observations()
    survival = 0.0
    vel_errors = []
    up_dots = []
    for _ in range(int(seconds / env.dt)):
        with torch.no_grad():
            mean, _ = teacher.policy.act_inference(
                torch.as_tensor(obs, dtype=torch.float32),
                torch.as_tensor(history, dtype=torch.float32),
            )
        obs, privileged, reward, done, infos, history = env.step(mean.numpy())
        survival += env.dt
        _ang, lin = env._base_velocity(0)
        vel_errors.append((vx - lin[0]) ** 2)
        rotation = env._base_rotation(0)
        up_dots.append(float(rotation[2, 2]))
        if done[0]:
            break
    return {
        "command": [vx, wz],
        "survival_s": float(survival),
        "tracking_vx_rmse": float(np.sqrt(np.mean(vel_errors))) if vel_errors else float("nan"),
        "mean_up_dot": float(np.mean(up_dots)) if up_dots else float("nan"),
        "final_height_m": float(env.data[0].qpos[2]),
    }


def evaluate_teacher(
    config: Config,
    onnx_path: str | Path,
    episodes_per_command: int = 2,
    seconds: float = 10.0,
    seed: int = 21,
) -> dict[str, Any]:
    dims = NetworkDims(num_privileged=141 + 4 * config.spring_count)
    teacher = restore_teacher(onnx_path, dims)

    results: dict[str, Any] = {}
    for name, (vx, wz) in EVAL_COMMANDS.items():
        episodes = [
            _episode(config, teacher, vx, wz, seconds, seed + index)
            for index in range(episodes_per_command)
        ]
        results[name] = {
            "command": [vx, wz],
            "mean_survival_s": float(np.mean([e["survival_s"] for e in episodes])),
            "tracking_vx_rmse": float(np.mean([e["tracking_vx_rmse"] for e in episodes])),
            "mean_up_dot": float(np.mean([e["mean_up_dot"] for e in episodes])),
            "episodes": episodes,
        }

    # The reproduction gate: the teacher must at least survive the standing
    # command on the target model for the evaluated horizon.  Anything better is
    # reported but not required, because the two robots differ in height.
    standing = results["stand"]
    passed = standing["mean_survival_s"] >= 0.9 * seconds
    return {
        "stage": "P6",
        "status": "passed" if passed else "failed",
        "teacher": teacher.report,
        "seconds_per_episode": seconds,
        "commands": results,
        "gate": "standing survival on the target model",
        "notes": [
            "target stance height 0.4648 m vs the teacher's training height range ~0.17-0.33 m",
            "channel order is still not proven; a failure here is not proof that the weights are wrong",
            "no camera/gimbal degrees of freedom are modelled",
        ],
    }

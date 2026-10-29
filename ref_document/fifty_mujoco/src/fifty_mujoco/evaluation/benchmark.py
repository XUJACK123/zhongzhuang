"""P8: measure policy-step throughput at increasing batch sizes."""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from ..config import Config
from ..envs.fifty_env import EnvOptions, FiftyVecEnv


def run_benchmark(config: Config, scales: list[int] | None = None, steps: int = 40) -> dict[str, Any]:
    scales = scales or [1, 16, 64]
    results: list[dict[str, Any]] = []
    for num_envs in scales:
        env = FiftyVecEnv(config, EnvOptions(num_envs=num_envs, seed=3))
        for _ in range(3):  # warm up
            env.step(np.zeros((num_envs, 6)))
        start = time.perf_counter()
        for _ in range(steps):
            env.step(np.zeros((num_envs, 6)))
        elapsed = time.perf_counter() - start
        results.append(
            {
                "num_envs": num_envs,
                "steps": steps,
                "wall_s": elapsed,
                "steps_per_s": steps / elapsed,
                "env_steps_per_s": num_envs * steps / elapsed,
                "physics_steps_per_s": num_envs * steps * env.decimation / elapsed,
            }
        )
        env.close()
    return {
        "stage": "P8",
        "backend": "cpu",
        "results": results,
        "note": "CPU numbers only; the GPU backend is not implemented yet (see P7)",
    }

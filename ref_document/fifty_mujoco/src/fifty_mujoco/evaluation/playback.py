"""P10: deterministic ONNX playback on the ordinary MuJoCo model."""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from ..config import Config
from ..envs.fifty_env import EnvOptions, FiftyVecEnv
from ..transfer.export import OnnxPolicy


def play_onnx(
    config: Config,
    onnx_path: str | Path,
    viewer: bool = False,
    seconds: float = 10.0,
    command: tuple[float, float] = (0.0, 0.0),
    variant: str = "visual",
) -> int:
    policy = OnnxPolicy(onnx_path)
    env = FiftyVecEnv(
        config,
        EnvOptions(num_envs=1, seed=0, variant=variant, fixed_command=(command[0], command[1], 0.464751)),
    )
    env.commands.values[:, 2] = env.nominal_height
    obs, history = env.get_observations()

    steps = int(seconds / env.dt)
    survived = 0
    print(f"  onnx    : {onnx_path}")
    print(f"  command : vx {command[0]:+.2f} m/s, wz {command[1]:+.2f} rad/s")
    print(f"  model   : {env.single.xml_path}")

    if viewer:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(env.model, env.data[0]) as handle:
            wall_start = time.time()
            for step in range(steps):
                if not handle.is_running():
                    break
                action = policy.act(obs, history)
                obs, privileged, reward, done, infos, history = env.step(action)
                survived += 1
                handle.sync()
                # real-time pacing
                target = wall_start + (step + 1) * env.dt
                delay = target - time.time()
                if delay > 0:
                    time.sleep(delay)
                if done[0]:
                    print(f"  episode ended at {survived * env.dt:.2f} s")
                    break
    else:
        for _ in range(steps):
            action = policy.act(obs, history)
            obs, privileged, reward, done, infos, history = env.step(action)
            survived += 1
            if done[0]:
                break

    print(f"  survived: {survived * env.dt:.2f} s of {seconds:.2f} s")
    print(f"  final height: {float(env.data[0].qpos[2]):.4f} m")
    return 0 if survived >= int(0.9 * steps) else 1

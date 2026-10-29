"""GPU environment: CPU/GPU parity and CUDA-graph replay consistency.

Skipped automatically when no CUDA device or ``mujoco_warp`` is available, so
the CPU-only part of the suite still runs on a machine without a GPU.
"""

from __future__ import annotations

import numpy as np
import pytest

from fifty_mujoco.config import load_configs


def _gpu_available() -> bool:
    try:
        import torch

        if not torch.cuda.is_available():
            return False
        import mujoco_warp  # noqa: F401

        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not _gpu_available(), reason="needs CUDA + mujoco_warp")


@pytest.fixture(scope="module")
def config():
    return load_configs()


@pytest.fixture(scope="module")
def height(config):
    from fifty_mujoco.control.spawn import load_spawn_spec, resolve_spawn_height
    from fifty_mujoco.envs.cpu_env import FiftyCpuEnv
    from fifty_mujoco.paths import assets_dir

    env = FiftyCpuEnv(assets_dir() / "mjcf" / "fifty_train.xml", config)
    root_z = resolve_spawn_height(env, load_spawn_spec(config))[0]
    return root_z - float(config.spawn["spawn"]["wheel_clearance_m"])


def test_gpu_matches_cpu_for_the_same_action_sequence(config, height):
    import torch

    from fifty_mujoco.envs.fifty_env import EnvOptions, FiftyVecEnv
    from fifty_mujoco.envs.warp_env import WarpEnvOptions, WarpVecEnv

    command = (0.0, 0.0, height)
    cpu = FiftyVecEnv(config, EnvOptions(num_envs=1, seed=1, fixed_command=command))
    gpu = WarpVecEnv(
        config, WarpEnvOptions(num_envs=1, device="cuda:0", seed=1, fixed_command=command)
    )
    actions = np.zeros((1, 6))
    for step in range(12):
        actions[0, 1] = 0.05 * np.sin(step / 3.0)
        actions[0, 4] = -actions[0, 1]
        actions[0, 2] = actions[0, 5] = 0.2
        cpu.step(actions)
        gpu.step(torch.as_tensor(actions, dtype=torch.float32, device=gpu.device))
    torch.cuda.synchronize()

    cpu_qpos = cpu.data[0].qpos
    gpu_qpos = gpu.qpos[0].cpu().numpy()
    assert abs(cpu_qpos[2] - gpu_qpos[2]) < 1e-4
    assert np.abs(cpu_qpos[7:] - gpu_qpos[7:]).max() < 1e-3


def test_graph_replay_matches_eager(config, height):
    """A captured step must produce exactly what the eager step produces."""
    import torch

    from fifty_mujoco.envs.warp_env import WarpEnvOptions, WarpVecEnv

    options = WarpEnvOptions(
        num_envs=4, device="cuda:0", seed=5, fixed_command=(0.0, 0.0, height)
    )
    eager = WarpVecEnv(config, options)
    captured = WarpVecEnv(config, options)
    assert captured.capture()

    actions = torch.zeros((4, 6), device=eager.device)
    actions[:, 2] = 0.2
    actions[:, 5] = 0.2
    for _ in range(10):
        eager.step(actions)
        captured.step(actions)
    torch.cuda.synchronize()
    assert torch.allclose(eager.qpos[:, 2], captured.qpos[:, 2], atol=1e-6)
    assert torch.allclose(eager.obs, captured.obs, atol=1e-5)


def test_capture_does_not_advance_the_simulation(config, height):
    from fifty_mujoco.envs.warp_env import WarpEnvOptions, WarpVecEnv

    env = WarpVecEnv(config, WarpEnvOptions(num_envs=8, device="cuda:0", seed=7))
    before = env.qpos[:, 2].clone()
    assert env.capture()
    after = env.qpos[:, 2]
    assert bool((before - after).abs().max() < 1e-6)


def test_illegal_contact_is_zero_when_standing(config, height):
    """The wheels are the only legal ground contact."""
    import torch

    from fifty_mujoco.envs.warp_env import WarpEnvOptions, WarpVecEnv

    env = WarpVecEnv(
        config,
        WarpEnvOptions(num_envs=4, device="cuda:0", seed=9, fixed_command=(0.0, 0.0, height)),
    )
    env.capture()
    env.step(torch.zeros((4, 6), device=env.device))
    torch.cuda.synchronize()
    assert float(env._illegal_contact_count().sum()) == pytest.approx(0.0)

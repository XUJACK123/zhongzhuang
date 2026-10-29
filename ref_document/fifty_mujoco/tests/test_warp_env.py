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


def _tilted(env, index: int = 0) -> None:
    import torch

    angle = torch.tensor(1.4)  # ~80 deg
    env.qpos[index, 3:7] = torch.tensor(
        [torch.cos(angle / 2), torch.sin(angle / 2), 0.0, 0.0], device=env.device
    )


def test_reset_zeroes_motor_control_and_returns_the_spawn_observation(config, height):
    """The advanced-index write is chained in the old reset and never lands."""
    import torch

    from fifty_mujoco.envs.warp_env import WarpEnvOptions, WarpVecEnv

    env = WarpVecEnv(
        config,
        WarpEnvOptions(num_envs=2, device="cuda:0", seed=11, fixed_command=(0.0, 0.0, height)),
    )
    actions = torch.ones((2, 6), device=env.device) * 0.5
    env.step(actions)
    env.reset()
    torch.cuda.synchronize()
    assert torch.allclose(env.ctrl[:, env.ctrl_idx], torch.zeros_like(env.ctrl[:, env.ctrl_idx]))
    assert torch.allclose(env.obs[:, 3:6], torch.tensor([0.0, 0.0, -1.0], device=env.device).expand(2, 3), atol=1e-3)
    assert torch.allclose(env.obs[:, 19:25], torch.zeros((2, 6), device=env.device))
    for index in range(2):
        for frame in env.history[index]:
            assert torch.allclose(frame, env.obs[index])


def test_terminal_snapshot_matches_the_terminal_frame(config, height):
    """terminal_history must end with the observation terminal_privileged embeds."""
    import torch

    from fifty_mujoco.envs.warp_env import WarpEnvOptions, WarpVecEnv

    env = WarpVecEnv(
        config,
        WarpEnvOptions(num_envs=1, device="cuda:0", seed=13, fixed_command=(0.0, 0.0, height)),
    )
    _tilted(env)
    for _ in range(40):
        _obs, _priv, _rew, done, _infos, _hist = env.step(
            torch.zeros((1, 6), device=env.device)
        )
        torch.cuda.synchronize()
        if bool(done[0]):
            terminal_hist = env.terminal_history[0].reshape(5, 25)
            terminal_priv = env.terminal_privileged[0]
            assert torch.allclose(terminal_hist[-1], terminal_priv[3 : 3 + 25], atol=1e-5)
            return
    pytest.fail("the forced tilt never terminated on the GPU env")


def test_spring_features_match_the_cpu_definition(config, height):
    """Same fields, unit and scaling as ``mdp.observations.spring_features``."""
    import torch

    from fifty_mujoco.control.gas_spring import reference_force
    from fifty_mujoco.contracts import names
    from fifty_mujoco.envs.fifty_env import EnvOptions, FiftyVecEnv
    from fifty_mujoco.envs.warp_env import WarpEnvOptions, WarpVecEnv

    command = (0.0, 0.0, height)
    cpu = FiftyVecEnv(config, EnvOptions(num_envs=1, seed=1, fixed_command=command))
    gpu = WarpVecEnv(
        config, WarpEnvOptions(num_envs=1, device="cuda:0", seed=1, fixed_command=command)
    )
    actions = torch.zeros((1, 6), device=gpu.device)
    for step in range(8):
        actions[0, 1] = 0.05 * np.sin(step / 3.0)
        actions[0, 4] = -actions[0, 1]
        row = actions.cpu().numpy()
        cpu.step(row)
        gpu.step(actions)
    torch.cuda.synchronize()
    if gpu.spring_count == 0:
        pytest.skip("no gas spring on this model")
    count = gpu.spring_count
    cpu_part = cpu.get_privileged_observations()[0, names.BASE_PRIVILEGED_DIM :]
    gpu_part = gpu.privileged[0, names.BASE_PRIVILEGED_DIM :].cpu().numpy()
    # Both backends now define the block identically - length from the current
    # geometry, rate over one policy step, force from the per-sub-step rate - so
    # all four channels per spring are comparable, not just length and force.
    assert np.allclose(cpu_part, gpu_part, atol=5e-3)
    assert np.allclose(cpu_part[:count], cpu.spring_lengths[0, :count] * 5.0, atol=1e-9)
    expected_reference = np.array([reference_force(spring) for spring in cpu.springs]) / 300.0
    assert np.allclose(cpu_part[2 * count : 3 * count], expected_reference, atol=1e-9)


def test_command_refresh_is_immediate(config, height):
    """``resample_commands`` must be visible before the next physics step."""
    import torch

    from fifty_mujoco.envs.warp_env import WarpEnvOptions, WarpVecEnv

    env = WarpVecEnv(
        config, WarpEnvOptions(num_envs=2, device="cuda:0", seed=17, fixed_command=(0.1, 0.0, height))
    )
    env.resample_commands()
    torch.cuda.synchronize()
    expected = env.commands * env.command_scale
    assert torch.allclose(env.obs[:, 6:9], expected, atol=1e-6)
    assert torch.allclose(env.history[:, -1, 6:9], expected, atol=1e-6)
    assert torch.allclose(env.privileged[:, 9:12], expected, atol=1e-6)

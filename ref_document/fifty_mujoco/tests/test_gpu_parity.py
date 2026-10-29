"""CPU/GPU contract parity as a regression test.

The full sweep lives in ``cli gpu-parity`` (4 envs x 24 steps + graph replay,
~20 s). This keeps a short version in the suite so the two defects it originally
caught cannot come back silently:

* the CPU env read ``data.cvel``/``mj_objectVelocity`` *after* ``mj_step``, which
  is one 5 ms integration stale, so ``obs[0:3]`` (and the reward terms that use
  the base velocity) disagreed with the GPU by up to ~1 rad/s;
* the CPU spring feature used the analytic ``J*qvel`` rate while the GPU used a
  finite difference, so the critic was trained on a different feature depending
  on the backend.
"""

from __future__ import annotations

import numpy as np
import pytest


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


def test_cpu_and_gpu_agree_field_by_field(config):
    from fifty_mujoco.evaluation.parity_suite import run_parity_suite

    report = run_parity_suite(config, num_envs=2, steps=6, seed=3, check_graph=False)
    assert report["status"] == "passed", report["failures"]
    assert report["termination_flag_mismatches"] == 0
    assert report["forced_reset"]["reached"] is True
    for name, entry in {**report["fields"], **report["field_groups"]}.items():
        assert entry["passed"], f"{name}: {entry}"
    # The observation groups that used to be stale must stay well inside budget.
    assert report["field_groups"]["obs.ang_vel"]["max_abs_error"] < 2e-3
    assert report["field_groups"]["history[4].ang_vel"]["max_abs_error"] < 2e-3


def test_base_velocity_reads_the_current_state(config):
    """``_base_velocity`` must match the free-joint ``qvel`` block, not ``cvel``."""
    from fifty_mujoco.control.spawn import load_spawn_spec, resolve_spawn_height
    from fifty_mujoco.envs.cpu_env import FiftyCpuEnv
    from fifty_mujoco.envs.fifty_env import EnvOptions, FiftyVecEnv
    from fifty_mujoco.paths import assets_dir

    probe = FiftyCpuEnv(assets_dir() / "mjcf" / "fifty_train.xml", config)
    height = resolve_spawn_height(probe, load_spawn_spec(config))[0] - float(
        config.spawn["spawn"]["wheel_clearance_m"]
    )
    env = FiftyVecEnv(config, EnvOptions(num_envs=1, seed=5, fixed_command=(0.0, 0.0, height)))
    env.step(np.array([[0.0, 0.0, 0.4, 0.0, 0.0, 0.4]]))
    data = env.data[0]
    dof = int(env.model.jnt_dofadr[int(env.model.body_jntadr[int(env.single.body_ids["base_link"])])])
    ang, lin = env._base_velocity(0)
    assert np.allclose(ang, data.qvel[dof + 3 : dof + 6], atol=1e-15)
    rotation = env._base_rotation(0)
    assert np.allclose(lin, rotation.T @ data.qvel[dof : dof + 3], atol=1e-15)

"""P7: batch-backend smoke test.

mjlab/MuJoCo-Warp is the intended GPU backend.  This module proves what is
actually available on this machine and refuses to pretend: a CPU-only run is
reported as ``cpu_only`` and the GPU gate stays unpassed.
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from ..config import Config
from ..control.spawn import load_spawn_spec, resolve_spawn_height
from ..envs.fifty_env import EnvOptions, FiftyVecEnv


def _cpu_batch(config: Config, num_envs: int, steps: int = 20) -> dict[str, Any]:
    env = FiftyVecEnv(config, EnvOptions(num_envs=num_envs, seed=13))
    start = time.perf_counter()
    finite = True
    for _ in range(steps):
        obs, privileged, reward, done, infos, history = env.step(np.zeros((num_envs, 6)))
        finite = finite and bool(np.all(np.isfinite(obs)) and np.all(np.isfinite(privileged)))
    elapsed = time.perf_counter() - start
    return {
        "num_envs": num_envs,
        "steps": steps,
        "wall_s": elapsed,
        "policy_steps_per_s": steps / elapsed,
        "env_steps_per_s": num_envs * steps / elapsed,
        "finite": finite,
        "reset_events": int(np.sum(infos["time_outs"])),
    }


def _mjw_batch(config: Config, nworld: int = 1024, steps: int = 100) -> dict[str, Any]:
    """Real GPU batch stepping through ``mujoco_warp``.

    This is the physics backend only: the PD law, observations and rewards are
    still NumPy in this project, so the end-to-end training loop is *not* on the
    GPU yet.  What is proven here is that the generated MJCF (meshes, tendons,
    tendon motors, contacts) runs N independent worlds on the device with finite
    state.
    """
    import mujoco
    import mujoco_warp as mjw
    import warp as warp

    from ..envs.cpu_env import FiftyCpuEnv
    from ..paths import assets_dir

    xml = assets_dir() / "mjcf" / "fifty_train.xml"
    cpu = FiftyCpuEnv(xml, config)
    root_z, _ = resolve_spawn_height(cpu, load_spawn_spec(config))
    defaults = np.array([config.default_angles[name] for name in config.joint_order])

    mjm = mujoco.MjModel.from_xml_path(str(xml))
    mjd = mujoco.MjData(mjm)
    qpos = np.zeros(mjm.nq)
    qpos[2] = root_z
    qpos[3] = 1.0
    qpos[7:] = defaults
    mjd.qpos[:] = qpos
    mjd.qvel[:] = 0.0
    mjd.ctrl[:] = 0.0
    spring_state = cpu.spring_state()
    for name, entry in spring_state.items():
        mjd.ctrl[cpu.spring_motor_ids[name]] = float(entry["force_n"])
    mujoco.mj_forward(mjm, mjd)

    model = mjw.put_model(mjm)
    data = mjw.put_data(mjm, mjd, nworld=nworld, nconmax=16 * nworld, njmax=32 * nworld)
    for _ in range(10):  # warm up / trigger kernel compilation
        mjw.step(model, data)
    warp.synchronize()

    start = time.perf_counter()
    for _ in range(steps):
        mjw.step(model, data)
    warp.synchronize()
    elapsed = time.perf_counter() - start
    qpos_out = data.qpos.numpy()
    heights = qpos_out[:, 2]
    return {
        "device": str(warp.get_device().name),
        "nworld": nworld,
        "steps": steps,
        "wall_s": elapsed,
        "env_steps_per_s": nworld * steps / elapsed,
        "finite": bool(np.all(np.isfinite(qpos_out))),
        "height_min_m": float(heights.min()),
        "height_max_m": float(heights.max()),
        "height_start_m": float(root_z),
    }


def run_smoke(config: Config, num_envs: int = 16) -> dict[str, Any]:
    checks: dict[str, Any] = {}
    note = None
    status = "failed"
    backend = "cpu"

    checks["cpu_batch"] = _cpu = _cpu_batch(config, num_envs)
    checks["cpu_batch"]["passed"] = bool(_cpu["finite"])

    gpu_report: dict[str, Any] = {}
    try:
        import mujoco
        import mjlab  # noqa: F401
        from mjlab.entity import EntityCfg  # noqa: F401

        gpu_report["mjlab_import"] = True
        from ..paths import assets_dir

        xml = assets_dir() / "mjcf" / "fifty_train.xml"
        spec = mujoco.MjSpec.from_file(str(xml))
        gpu_report["mj_spec_bodies"] = len(spec.bodies)
        gpu_report["mj_spec_joints"] = len(spec.joints)
        gpu_report["mj_spec_actuators"] = len(spec.actuators)
        checks["mjlab_spec"] = {
            "passed": gpu_report["mj_spec_bodies"] >= 7,
            "bodies": gpu_report["mj_spec_bodies"],
            "joints": gpu_report["mj_spec_joints"],
            "actuators": gpu_report["mj_spec_actuators"],
        }
        backend = "mjlab_available"
        try:
            batch = _mjw_batch(config)
            batch["passed"] = bool(
                batch["finite"] and batch["height_min_m"] > 0.0 and batch["env_steps_per_s"] > 0.0
            )
            checks["mjw_batch"] = batch
            gpu_report["mjw_batch"] = batch
            note = (
                "GPU physics batch verified through mujoco_warp "
                f"({batch['env_steps_per_s']:.0f} env-steps/s at {batch['nworld']} worlds). "
                "PD, observations and rewards still run in NumPy, and no "
                "ManagerBasedRlEnv was built, so end-to-end GPU training is NOT "
                "verified: the GPU training gate stays unpassed."
            )
            status = "partial"
        except Exception as error:  # pragma: no cover - depends on the install
            gpu_report["mjw_error"] = f"{type(error).__name__}: {error}"
            note = (
                "mjlab imports and the MJCF compiles into an MjSpec, but the "
                f"mujoco_warp batch failed ({type(error).__name__}); only the CPU "
                "batch backend is verified.  The GPU gate stays unpassed."
            )
            status = "partial"
    except Exception as error:  # pragma: no cover - depends on the install
        gpu_report["mjlab_import"] = False
        gpu_report["error"] = f"{type(error).__name__}: {error}"
        note = (
            "mjlab is unavailable, so only the CPU batch backend was exercised. "
            "The GPU gate stays unpassed."
        )
        status = "cpu_only"

    return {
        "stage": "P7",
        "status": status,
        "backend": backend,
        "num_envs": num_envs,
        "gpu": gpu_report,
        "checks": checks,
        "note": note,
        "passed": status == "passed",
    }

"""P4: per-channel sign / unit / saturation test (doc section 4.3)."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..config import Config
from ..contracts import names
from ..envs.cpu_env import FiftyCpuEnv
from ..control.mixed_pd import PdGains, compute_torques, decode_action
from ..control.spawn import load_spawn_spec, resolve_spawn_height
from ..mdp import observations as obs_lib


def _hold_base(env: FiftyCpuEnv, index_data) -> None:
    index_data.qvel[0:6] = 0.0


def run_joint_test(config: Config, env: FiftyCpuEnv, steps: int = 40) -> dict[str, Any]:
    """Excite one channel at a time with a small command and record the response.

    Isolation matters: the passive spring produces ~9 N.m at the knee, far more
    than the ~1 N.m that a 0.05 rad target offset produces, so the tendon motors
    are switched off here.

    The horizon is deliberately short (3 physics steps for legs, 1 for wheels).
    With the base pinned the leg is unloaded (effective knee inertia ~1e-3
    kg m^2); the explicit PD (Kp=20, Kd=1, dt=5 ms) is numerically unstable in
    that unloaded case and diverges after ~5 steps.  The sign of the first steps
    is what this test is for; a standing robot loads the leg through its body.
    """
    spec = load_spawn_spec(config)
    root_z, _ = resolve_spawn_height(env, spec)
    gains = PdGains(
        leg_kp=float(config.robot["control"]["leg_kp"]),
        leg_kd=float(config.robot["control"]["leg_kd"]),
        wheel_kp=float(config.robot["control"]["wheel_kp"]),
        wheel_kd=float(config.robot["control"]["wheel_kd"]),
        torque_limit_nm=config.effort_limit("rf0_Joint"),
        wheel_torque_limit_nm=config.effort_limit("r_wheel_Joint"),
    )
    gravity = env.model.opt.gravity.copy()
    env.model.opt.gravity[:] = 0.0  # isolate the joint response from gravity

    results: dict[str, Any] = {}
    try:
        for joint_name in names.MODEL_JOINT_ORDER:
            is_wheel = joint_name in names.WHEEL_JOINTS
            amplitude = 1.0 if is_wheel else 0.05
            horizon = 1 if is_wheel else 3
            for sign in (+1.0, -1.0):
                env.reset(root_position=np.array([0.0, 0.0, root_z]))
                for actuator_id in env.spring_motor_ids.values():
                    env.data.ctrl[actuator_id] = 0.0  # isolate the motor channel
                action = np.zeros(6)
                index = names.MODEL_JOINT_ORDER.index(joint_name)
                action[index] = sign * amplitude
                start_position = float(env.data.qpos[env.qpos_adr[joint_name]])
                start_velocity = float(env.data.qvel[env.dof_adr[joint_name]])
                peak_torque = 0.0
                saturated = 0
                for _ in range(horizon):
                    position = np.array(
                        [env.data.qpos[env.qpos_adr[name]] for name in names.MODEL_JOINT_ORDER]
                    )
                    velocity = np.array(
                        [env.data.qvel[env.dof_adr[name]] for name in names.MODEL_JOINT_ORDER]
                    )
                    targets = decode_action(action, config.default_angles, gains)
                    torque = compute_torques(targets, position, velocity, gains, motor_enabled=True)
                    peak_torque = max(peak_torque, float(np.abs(torque[index])))
                    saturated += int(abs(abs(torque[index]) - gains.limits()[index]) < 1e-9)
                    for name, value in zip(names.MODEL_JOINT_ORDER, torque, strict=True):
                        env.data.ctrl[env.joint_motor_ids[name]] = float(value)
                    for actuator_id in env.spring_motor_ids.values():
                        env.data.ctrl[actuator_id] = 0.0
                    _hold_base(env, env.data)
                    env.step()
                end_position = float(env.data.qpos[env.qpos_adr[joint_name]])
                end_velocity = float(env.data.qvel[env.dof_adr[joint_name]])
                delta_position = end_position - start_position
                delta_velocity = end_velocity - start_velocity

                if is_wheel:
                    moved_correctly = np.sign(delta_velocity) == np.sign(sign)
                    measured = delta_velocity
                    unit = "rad/s"
                else:
                    moved_correctly = np.sign(delta_position) == np.sign(sign)
                    measured = delta_position
                    unit = "rad"
                key = f"{joint_name}_{'plus' if sign > 0 else 'minus'}"
                results[key] = {
                    "joint": joint_name,
                    "command": float(action[index]),
                    "measured": float(measured),
                    "unit": unit,
                    "peak_torque_nm": peak_torque,
                    "saturated_steps": saturated,
                    "sign_ok": bool(moved_correctly),
                }
    finally:
        env.model.opt.gravity[:] = gravity

    all_ok = all(entry["sign_ok"] for entry in results.values())
    return {
        "stage": "P4",
        "status": "passed" if all_ok else "failed",
        "gravity_disabled": True,
        "springs_disabled": True,
        "base_held": True,
        "wheel_horizon_steps": 1,
        "leg_horizon_steps": 3,
        "note": (
            "short horizon on purpose: with a pinned base the leg is unloaded and the "
            "explicit PD diverges after ~5 steps (documented in reports/joint_test.json)"
        ),
        "channels": results,
        "peak_torque_nm": max(entry["peak_torque_nm"] for entry in results.values()),
    }


def run_observation_probe(config: Config, env: FiftyCpuEnv) -> dict[str, Any]:
    """Sanity numbers that the contract test asserts against."""
    spec = load_spawn_spec(config)
    root_z, _ = resolve_spawn_height(env, spec)
    env.reset(root_position=np.array([0.0, 0.0, root_z]))
    obs = obs_lib.proprio(
        np.zeros(3),
        np.array([0.0, 0.0, -1.0]),
        np.zeros(3),
        np.array([config.default_angles[name] for name in names.MODEL_JOINT_ORDER]),
        np.zeros(6),
        np.zeros(6),
        np.array([config.default_angles[name] for name in names.MODEL_JOINT_ORDER]),
    )
    return {"upright_observation": obs.tolist(), "size": int(obs.size)}

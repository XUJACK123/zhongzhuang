"""P4: gas-spring vs motor combinations A/B/C/D (doc section 12.6)."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..config import Config
from ..contracts import names
from ..envs.cpu_env import FiftyCpuEnv
from ..control.spawn import load_spawn_spec, resolve_spawn_height


def _run(config: Config, env: FiftyCpuEnv, springs: bool, motors: bool, root_z: float, duration_s: float) -> dict[str, Any]:
    spec = load_spawn_spec(config)
    env.reset(root_position=np.array([0.0, 0.0, root_z]))
    steps = int(duration_s / float(env.model.opt.timestep))
    torque_squares = []
    saturated = 0
    peak_spring = 0.0
    heights = []

    for _ in range(steps):
        if motors:
            for joint_name in names.MODEL_JOINT_ORDER:
                position = float(env.data.qpos[env.qpos_adr[joint_name]])
                velocity = float(env.data.qvel[env.dof_adr[joint_name]])
                target = config.default_angles[joint_name]
                limit = config.effort_limit(joint_name)
                kp = 0.0 if joint_name in names.WHEEL_JOINTS else float(config.robot["control"]["leg_kp"])
                kd = float(config.robot["control"]["wheel_kd"]) if joint_name in names.WHEEL_JOINTS else float(config.robot["control"]["leg_kd"])
                torque = float(np.clip(kp * (target - position) - kd * velocity, -limit, limit))
                env.data.ctrl[env.joint_motor_ids[joint_name]] = torque
                torque_squares.append(torque**2)
                saturated += int(abs(abs(torque) - limit) < 1e-9)
        else:
            env.zero_joint_motors()

        if springs:
            state = env.apply_passive_springs()
            peak_spring = max(peak_spring, max(entry["force_n"] for entry in state.values()))
        else:
            for actuator_id in env.spring_motor_ids.values():
                env.data.ctrl[actuator_id] = 0.0

        env.step()
        heights.append(float(env.data.qpos[2]))

    rotation = np.zeros(9)
    import mujoco

    mujoco.mju_quat2Mat(rotation, env.data.qpos[3:7] / np.linalg.norm(env.data.qpos[3:7]))
    up_dot = float(rotation.reshape(3, 3)[2, 2])
    torque_count = max(1, len(torque_squares))
    return {
        "springs": springs,
        "motors": motors,
        "start_height_m": float(root_z),
        "final_height_m": float(env.data.qpos[2]),
        "min_height_m": float(min(heights)),
        "up_dot": up_dot,
        "torque_rms_nm": float(np.sqrt(np.sum(torque_squares) / torque_count)),
        "saturation_fraction": saturated / max(1, torque_count),
        "max_spring_force_n": peak_spring,
    }


def run_spring_sweep(config: Config, env: FiftyCpuEnv, duration_s: float = 1.0) -> dict[str, Any]:
    spec = load_spawn_spec(config)
    root_z, _ = resolve_spawn_height(env, spec)
    combinations = {
        "A_springs_off_motors_off": (False, False),
        "B_springs_on_motors_off": (True, False),
        "C_springs_on_motors_on": (True, True),
        "D_springs_off_motors_on": (False, True),
    }
    results = {
        name: _run(config, env, springs, motors, root_z, duration_s)
        for name, (springs, motors) in combinations.items()
    }
    checks = {
        "passive_force_without_motors": {
            "pass_motor_off_force_n": results["B_springs_on_motors_off"]["max_spring_force_n"],
            "no_motor_force_n": results["A_springs_off_motors_off"]["max_spring_force_n"],
            "passed": results["B_springs_on_motors_off"]["max_spring_force_n"] > 0.0
            and results["A_springs_off_motors_off"]["max_spring_force_n"] == 0.0,
        },
        "springs_improve_height_without_motors": {
            "with_springs_m": results["B_springs_on_motors_off"]["final_height_m"],
            "without_springs_m": results["A_springs_off_motors_off"]["final_height_m"],
            "passed": results["B_springs_on_motors_off"]["final_height_m"]
            > results["A_springs_off_motors_off"]["final_height_m"],
        },
        "motors_can_hold_with_springs": {
            "final_height_m": results["C_springs_on_motors_on"]["final_height_m"],
            "passed": results["C_springs_on_motors_on"]["final_height_m"] > 0.6 * root_z,
            "note": "PD-only hold on a balancing robot; not a standing-policy check",
        },
    }
    return {
        "stage": "P4",
        "status": "passed" if all(check["passed"] for check in checks.values()) else "failed",
        "duration_s": duration_s,
        "combinations": results,
        "checks": checks,
        "note": "A is an ablation only; B is the phase-1 training default; C is an experiment",
    }

"""P5: contact behaviour (doc section 6.3)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from ..config import Config
from ..contracts import names
from ..envs.cpu_env import FiftyCpuEnv
from ..control.spawn import load_spawn_spec, resolve_spawn_height


def contact_forces(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, Any]]:
    """Normal force per contact, from the constraint solver output."""
    result: list[dict[str, Any]] = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        address = int(contact.efc_address)
        if address < 0:
            continue
        body_a = int(model.geom_bodyid[contact.geom1])
        body_b = int(model.geom_bodyid[contact.geom2])
        name_a = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_a) or "world"
        name_b = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_b) or "world"
        result.append(
            {
                "bodies": (name_a, name_b),
                "normal_force_n": float(data.efc_force[address]),
                "penetration_m": float(contact.dist),
            }
        )
    return result


def _pin_base(env: FiftyCpuEnv) -> None:
    env.data.qvel[0:6] = 0.0


def _vertical_slider(env: FiftyCpuEnv) -> None:
    """Attitude and horizontal position pinned, vertical motion free.

    This is the minimal external support that lets a two-wheel inverted
    pendulum reach a *static* vertical equilibrium: the pin can only carry
    torque and horizontal force, never weight, so the summed wheel normal force
    must equal the weight.  Without it the rig simply tips over (no balancing
    controller exists at this stage), which is what made the earlier version of
    this test read 0 N.
    """
    env.data.qpos[0:2] = 0.0
    env.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    env.data.qvel[0:2] = 0.0
    env.data.qvel[3:6] = 0.0


@dataclass
class ContactConfig:
    load_steps: int = 300
    drop_steps: int = 200
    roll_steps: int = 20
    roll_targets: tuple[float, ...] = (2.0, 0.0, -2.0)


def _hold_legs(env: FiftyCpuEnv, config: Config, springs: bool = False) -> None:
    """PD hold at the default pose (no policy involved)."""
    if springs:
        env.apply_passive_springs()
    else:
        for actuator_id in env.spring_motor_ids.values():
            env.data.ctrl[actuator_id] = 0.0
    for joint_name in names.MODEL_JOINT_ORDER:
        position = float(env.data.qpos[env.qpos_adr[joint_name]])
        velocity = float(env.data.qvel[env.dof_adr[joint_name]])
        target = config.default_angles[joint_name]
        limit = config.effort_limit(joint_name)
        if joint_name in names.WHEEL_JOINTS:
            torque = float(np.clip(-0.2 * velocity, -limit, limit))
        else:
            torque = float(
                np.clip(
                    float(config.robot["control"]["leg_kp"]) * (target - position)
                    - float(config.robot["control"]["leg_kd"]) * velocity,
                    -limit,
                    limit,
                )
            )
        env.data.ctrl[env.joint_motor_ids[joint_name]] = torque


def run_contact_test(config: Config, env: FiftyCpuEnv, steps: int = 200) -> dict[str, Any]:
    spec = load_spawn_spec(config)
    root_z, bottoms = resolve_spawn_height(env, spec)
    clearance = float(config.spawn["spawn"]["wheel_clearance_m"])
    grounded_z = root_z - clearance + 0.001  # wheels just touching
    total_mass = float(sum(env.model.body_mass[1:]))
    weight = total_mass * 9.81
    checks: dict[str, Any] = {}

    # ---------------------------------------------------------- dual wheel load
    env.reset(root_position=np.array([0.0, 0.0, grounded_z]))
    totals = []
    for step in range(ContactConfig.load_steps):
        _vertical_slider(env)
        _hold_legs(env, config, springs=True)
        env.step()
        if step >= ContactConfig.load_steps - 100:
            totals.append(
                sum(
                    f["normal_force_n"]
                    for f in contact_forces(env.model, env.data)
                    if "wheel" in (f["bodies"][0] + f["bodies"][1])
                )
            )
    forces = contact_forces(env.model, env.data)
    wheel_forces = [
        f["normal_force_n"]
        for f in forces
        if "wheel" in (f["bodies"][0] + f["bodies"][1])
    ]
    vertical = float(np.mean(totals))
    checks["dual_wheel_load"] = {
        "wheel_normal_force_n": wheel_forces,
        "total_vertical_n": vertical,
        "weight_n": weight,
        "ratio": vertical / weight if weight else 0.0,
        "settled_height_m": float(env.data.qpos[2]),
        "passed": 0.95 <= (vertical / weight if weight else 0.0) <= 1.05,
        "note": "vertical-slider support (no weight carried by the pin); mean of the last 100 steps",
    }

    # ------------------------------------------------------------------- drop
    env.reset(root_position=np.array([0.0, 0.0, root_z + 0.3]))
    peak = 0.0
    for _ in range(steps):
        env.zero_joint_motors()
        env.step()
        for force in contact_forces(env.model, env.data):
            peak = max(peak, force["normal_force_n"])
    checks["drop"] = {
        "start_height_m": float(root_z + 0.3),
        "final_height_m": float(env.data.qpos[2]),
        "peak_contact_force_n": peak,
        "finite": bool(np.all(np.isfinite(env.data.qpos))),
        "passed": bool(np.all(np.isfinite(env.data.qpos))) and peak > 0.0,
        "note": "no controller: falling over after landing is expected",
    }

    # ---------------------------------------------------------------- rolling
    # The wheel channels are measured on a FREE base over 0.1 s only.  A two
    # wheel inverted pendulum without a balancing controller tips over within
    # ~0.2 s, and from then on the joint velocity is dominated by the body pitch,
    # not by rolling.  What is checked here is that the drive produces rotation
    # in the commanded direction with the correct ordering across commands.
    measured: dict[float, float] = {}
    for target in ContactConfig.roll_targets:
        env.reset(root_position=np.array([0.0, 0.0, grounded_z]))
        samples = []
        for step in range(ContactConfig.roll_steps):
            _hold_legs(env, config)
            for joint_name in names.WHEEL_JOINTS:
                velocity = float(env.data.qvel[env.dof_adr[joint_name]])
                limit = config.effort_limit(joint_name)
                env.data.ctrl[env.joint_motor_ids[joint_name]] = float(
                    np.clip(0.2 * (target - velocity), -limit, limit)
                )
            env.step()
            if step >= 5:
                samples.append(
                    [float(env.data.qvel[env.dof_adr[name]]) for name in names.WHEEL_JOINTS]
                )
        measured[target] = float(np.mean(np.asarray(samples))) if samples else 0.0
    forward, neutral, backward = (measured[t] for t in (2.0, 0.0, -2.0))
    checks["rolling"] = {
        "mean_wheel_rad_s": {f"{k:+.1f}": v for k, v in measured.items()},
        "window_s": 0.075,
        "passed": bool(forward > 0.0 and backward < 0.0 and forward > neutral > backward),
        "note": "free base, gas springs off, first 0.1 s; travel distance is excluded "
        "because it is dominated by the uncontrolled tip-over",
    }

    # ------------------------------------------------------------- fallen pose
    env.reset(root_position=np.array([0.0, 0.0, 0.16]))
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.array([np.pi / 2, 0.0, 0.0]), "xyz")
    env.data.qpos[3:7] = quat
    env.forward()
    for _ in range(5):
        env.zero_joint_motors()
        env.step()
    from ..mdp.terminations import illegal_contacts

    illegal = illegal_contacts(env.model, env.data, {env.body_ids["right_wheel"], env.body_ids["left_wheel"]})
    checks["fallen_pose_detected"] = {
        "illegal_bodies": illegal,
        "passed": bool(illegal),
    }

    return {
        "stage": "P5",
        "status": "passed" if all(check["passed"] for check in checks.values()) else "failed",
        "total_mass_kg": total_mass,
        "checks": checks,
    }

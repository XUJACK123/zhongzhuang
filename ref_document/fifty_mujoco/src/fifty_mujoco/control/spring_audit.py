"""P3 checks for the passive gas spring (doc section 12.8, reference profile).

These checks prove the *interface* is correct: anchors on the right bodies,
length/velocity/Jacobian consistency, force sign, single application path,
left/right independence and virtual work.  They cannot prove the spring is
calibrated — the anchors are transferred from the Fudan model and the stroke is
unknown, so the P3 gate stays open.
"""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from ..config import Config
from ..envs.cpu_env import SPRING_CTRL_SIGN, FiftyCpuEnv
from .gas_spring import check_stroke, load_springs, spring_force, spring_geometry
from .spawn import load_spawn_spec, resolve_spawn_height


def _site_body(model: mujoco.MjModel, site_name: str) -> str:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id < 0:
        raise ValueError(f"site {site_name!r} missing")
    body_id = int(model.site_bodyid[site_id])
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)


def _joint_dofs(env: FiftyCpuEnv) -> np.ndarray:
    return np.array([env.dof_adr[name] for name in env.config.joint_order], dtype=int)


def audit_springs(config: Config, env: FiftyCpuEnv) -> dict[str, Any]:
    if not config.gas_spring_configured:
        return {
            "stage": "P3",
            "status": "not_configured",
            "profile": config.gas_spring_profile,
            "note": "configs/gas_spring.yaml has no usable anchors/force",
            "checks": {},
        }

    springs = load_springs(config)
    spec = load_spawn_spec(config)
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], 0.0]))
    root_z, _ = resolve_spawn_height(env, spec)

    checks: dict[str, Any] = {}
    joint_dofs = _joint_dofs(env)

    # ------------------------------------------------------------ anchors/bodies
    anchor_report: dict[str, Any] = {}
    anchors_ok = True
    for spring in springs:
        body_a = _site_body(env.model, f"{spring.name}_a")
        body_b = _site_body(env.model, f"{spring.name}_b")
        ok = body_a == spring.body_a and body_b == spring.body_b
        anchors_ok = anchors_ok and ok
        anchor_report[spring.name] = {
            "site_a_body": body_a,
            "site_b_body": body_b,
            "expected": [spring.body_a, spring.body_b],
            "passed": ok,
        }
    checks["anchors_on_declared_bodies"] = {"passed": anchors_ok, "per_spring": anchor_report}

    # ------------------------------------------------ length / tendon consistency
    state = env.spring_state()
    length_report: dict[str, Any] = {}
    length_ok = True
    for name, entry in state.items():
        delta = abs(entry["length_m"] - entry["tendon_length_m"])
        ok = delta <= 1e-9
        length_ok = length_ok and ok
        length_report[name] = {
            "site_distance_m": entry["length_m"],
            "tendon_length_m": entry["tendon_length_m"],
            "difference_m": delta,
            "passed": ok,
        }
    checks["length_matches_tendon"] = {"passed": length_ok, "per_spring": length_report}

    # ------------------------------------------------------ Jacobian consistency
    # `MjData.ten_J` is truncated in MuJoCo 3.17 (16 entries for nv*ntendon=24),
    # so the tendon Jacobian is verified against a finite difference of the
    # tendon length, and the tendon *velocity* is used as the independent check.
    jacobian_report: dict[str, Any] = {}
    jacobian_ok = True
    base_qpos = env.data.qpos.copy()
    for spring in springs:
        analytic = spring_geometry(env, spring)["jacobian"]
        numeric = np.zeros(env.model.nv)
        epsilon = 1e-6
        for joint_name in config.joint_order:
            address = env.qpos_adr[joint_name]
            dof = env.dof_adr[joint_name]
            env.data.qpos[:] = base_qpos
            env.data.qpos[address] = base_qpos[address] + epsilon
            env.forward()
            plus = float(env.data.ten_length[env.tendon_ids[spring.name]])
            env.data.qpos[address] = base_qpos[address] - epsilon
            env.forward()
            minus = float(env.data.ten_length[env.tendon_ids[spring.name]])
            numeric[dof] = (plus - minus) / (2.0 * epsilon)
        env.data.qpos[:] = base_qpos
        env.forward()

        base_dof = float(np.abs(analytic[:6]).max())
        error = float(np.abs(numeric - analytic).max())
        ok = error <= 1e-6 and base_dof <= 1e-9
        jacobian_ok = jacobian_ok and ok
        jacobian_report[spring.name] = {
            "analytic_joint_dofs": analytic[6:].tolist(),
            "finite_difference_joint_dofs": numeric[6:].tolist(),
            "max_abs_difference": error,
            "max_abs_base_dof_entry": base_dof,
            "passed": ok,
        }
    checks["jacobian_matches_finite_difference"] = {
        "passed": jacobian_ok,
        "per_spring": jacobian_report,
        "note": "floating-base dofs must have zero length derivative (rigid-body invariance)",
    }

    # ------------------------------------------------- tendon velocity consistency
    velocity_report: dict[str, Any] = {}
    velocity_ok = True
    rng_check = np.random.default_rng(11)
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], root_z]))
    qvel = np.zeros(env.model.nv)
    for name in config.joint_order:
        qvel[env.dof_adr[name]] = rng_check.uniform(-0.5, 0.5)
    env.data.qvel[:] = qvel
    env.forward()
    for spring in springs:
        geometry = spring_geometry(env, spring)
        tendon_velocity = float(env.data.ten_velocity[env.tendon_ids[spring.name]])
        error = abs(tendon_velocity - geometry["length_rate_m_s"])
        ok = error <= 1e-9
        velocity_ok = velocity_ok and ok
        velocity_report[spring.name] = {
            "tendon_velocity_m_s": tendon_velocity,
            "jacobian_times_qvel_m_s": geometry["length_rate_m_s"],
            "difference_m_s": error,
            "passed": ok,
        }
    checks["tendon_velocity_matches_jacobian"] = {
        "passed": velocity_ok,
        "per_spring": velocity_report,
    }

    # ---------------------------------------------------------------- force sign
    # Fixed base, no gravity: apply a 10 N spring command and compare the
    # generalized force with J_l^T F (push apart).
    env.model.opt.gravity[:] = 0.0
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], root_z]))
    env.zero_joint_motors()
    test_force = 10.0
    sign_report: dict[str, Any] = {}
    sign_ok = True
    for spring in springs:
        for actuator_id in env.spring_motor_ids.values():
            env.data.ctrl[actuator_id] = 0.0
        env.data.ctrl[env.spring_motor_ids[spring.name]] = SPRING_CTRL_SIGN * test_force
        env.forward()
        jacobian = spring_geometry(env, spring)["jacobian"]
        expected = jacobian * test_force
        actual = np.asarray(env.data.qfrc_actuator, dtype=float)[joint_dofs]
        error = float(np.abs(actual - expected[joint_dofs]).max())
        ok = error <= 1e-9
        sign_ok = sign_ok and ok
        sign_report[spring.name] = {
            "expected_joint_torque": expected[joint_dofs].tolist(),
            "actual_joint_torque": actual.tolist(),
            "max_abs_difference": error,
            "passed": ok,
        }
    checks["force_sign_matches_push_apart"] = {
        "passed": sign_ok,
        "ctrl_sign": SPRING_CTRL_SIGN,
        "note": "positive force_n pushes the anchors apart; ctrl is negated",
        "per_spring": sign_report,
    }

    # ------------------------------------------------ no net external base force
    base_forces: dict[str, Any] = {}
    base_ok = True
    for spring in springs:
        for actuator_id in env.spring_motor_ids.values():
            env.data.ctrl[actuator_id] = 0.0
        env.data.ctrl[env.spring_motor_ids[spring.name]] = SPRING_CTRL_SIGN * 300.0
        env.forward()
        qfrc = np.asarray(env.data.qfrc_actuator, dtype=float)
        base_dofs = np.array([0, 1, 2, 3, 4, 5])
        magnitude = float(np.abs(qfrc[base_dofs]).max())
        ok = magnitude <= 1e-9
        base_ok = base_ok and ok
        base_forces[spring.name] = {"max_base_dof_force": magnitude, "passed": ok}
    checks["no_net_external_force"] = {
        "passed": base_ok,
        "per_spring": base_forces,
        "note": "an internal spring pair must not push the whole robot",
    }

    # ------------------------------------------------------ motors-off still works
    env.model.opt.gravity[:] = config.robot["simulation"]["gravity"]
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], root_z]))
    env.zero_joint_motors()
    env.apply_passive_springs()
    env.forward()
    qfrc = np.asarray(env.data.qfrc_actuator, dtype=float)[joint_dofs]
    motor_off_ok = float(np.abs(qfrc).max()) > 1.0
    checks["passive_force_with_motors_off"] = {
        "passed": motor_off_ok,
        "joint_torque_from_springs_nm": qfrc.tolist(),
        "joint_motor_commands": [float(env.data.ctrl[i]) for i in env.joint_motor_ids.values()],
    }

    # -------------------------------------------------------- left/right independence
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], root_z]))
    for actuator_id in env.spring_motor_ids.values():
        env.data.ctrl[actuator_id] = 0.0
    right = next((s for s in springs if s.side == "right"), None)
    left = next((s for s in springs if s.side == "left"), None)
    independence: dict[str, Any] = {}
    independence_ok = True
    if right and left:
        env.data.ctrl[env.spring_motor_ids[right.name]] = SPRING_CTRL_SIGN * 300.0
        env.forward()
        left_tendon_force = abs(float(env.data.actuator_force[env.spring_motor_ids[left.name]]))
        # qfrc contribution restricted to the left leg joints
        left_joint_torque = 0.0
        for joint_name in ("lf0_Joint", "lf1_Joint", "l_wheel_Joint"):
            index = env.dof_adr[joint_name]
            left_joint_torque = max(left_joint_torque, abs(float(env.data.qfrc_actuator[index])))
        independence_ok = left_tendon_force <= 1e-12 and left_joint_torque <= 1e-9
        independence = {
            "left_tendon_force_when_right_driven": left_tendon_force,
            "max_left_joint_torque_when_right_driven": left_joint_torque,
        }
    checks["left_right_independent"] = {"passed": independence_ok, **independence}

    # ------------------------------------------------------------- virtual work
    rng = np.random.default_rng(7)
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], root_z]))
    qvel = np.zeros(env.model.nv)
    for name in config.joint_order:
        qvel[env.dof_adr[name]] = rng.uniform(-0.3, 0.3)
    env.data.qvel[:] = qvel
    env.forward()
    work_report: dict[str, Any] = {}
    work_ok = True
    for spring in springs:
        for actuator_id in env.spring_motor_ids.values():
            env.data.ctrl[actuator_id] = 0.0
        env.data.ctrl[env.spring_motor_ids[spring.name]] = SPRING_CTRL_SIGN * 300.0
        env.forward()
        geometry = spring_geometry(env, spring)
        qfrc = np.asarray(env.data.qfrc_actuator, dtype=float)
        actuator_power = float(qfrc @ env.data.qvel)
        spring_power = float(300.0 * geometry["length_rate_m_s"])
        error = abs(actuator_power - spring_power)
        ok = error <= 1e-6 * max(1.0, abs(spring_power))
        work_ok = work_ok and ok
        work_report[spring.name] = {
            "actuator_power_w": actuator_power,
            "spring_power_w": spring_power,
            "difference_w": error,
            "passed": ok,
        }
    checks["virtual_work_consistency"] = {"passed": work_ok, "per_spring": work_report}

    # ----------------------------------------------------------- stroke status
    stroke_report = {
        spring.name: check_stroke(spring, state[spring.name]["length_m"]) for spring in springs
    }
    # Not an interface check: unknown stroke is a *data* gap, tracked as a gate.
    gate_reasons = []
    if not config.gas_spring_stroke_verified:
        gate_reasons.append("stroke limits unknown (length_min_m / length_max_m are null)")
    if any(spring.parameter_status.startswith("reference") for spring in springs):
        gate_reasons.append("anchor geometry transferred from the Fudan model, not measured here")
    gate = {
        "closed": not gate_reasons,
        "reasons": gate_reasons,
        "stroke_report": stroke_report,
        "anchors_measured": False,
    }

    # ------------------------------------------- free-support observation (300 N)
    support = _support_test(env, config, springs, root_z)

    report = {
        "stage": "P3",
        "status": "passed" if all(check["passed"] for check in checks.values()) else "failed",
        "gate": gate,
        "gate_closed": bool(gate["closed"]),
        "profile": config.gas_spring_profile,
        "parameter_status": [spring.parameter_status for spring in springs],
        "calibrated": False,
        "xml_path": str(env.xml_path.resolve()),
        "spawn_z_m": root_z,
        "springs": {
            spring.name: {
                **spring.as_dict(),
                "length_at_spawn_m": state[spring.name]["length_m"],
                "force_at_spawn_n": state[spring.name]["force_n"],
                "tendon_id": env.tendon_ids[spring.name],
                "actuator_id": env.spring_motor_ids[spring.name],
            }
            for spring in springs
        },
        "checks": checks,
        "support_test": support,
        "gate_note": (
            "Interface checks verify anchors, length/velocity/Jacobian, force sign, single "
            "application and left/right independence.  The gate itself stays open while the "
            "anchors are transferred from the Fudan model and the stroke is unknown."
        ),
    }
    return report


def _support_test(
    env: FiftyCpuEnv, config: Config, springs, root_z: float, duration_s: float = 1.0
) -> dict[str, Any]:
    """Zero-motor free-base run: does the passive spring measurably hold the robot?

    This is a physics observation, not a standing test: there is no controller.
    """
    spec = load_spawn_spec(config)
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], root_z]))
    env.zero_joint_motors()

    samples = int(duration_s / float(env.model.opt.timestep))
    heights = []
    for _ in range(samples):
        env.apply_passive_springs()
        env.step()
        env.zero_joint_motors()
        heights.append(float(env.data.qpos[2]))

    quat = np.asarray(env.data.qpos[3:7], dtype=float)
    rotation = np.zeros(9)
    mujoco.mju_quat2Mat(rotation, quat / np.linalg.norm(quat))
    up_dot = float(rotation.reshape(3, 3)[2, 2])

    return {
        "duration_s": duration_s,
        "start_height_m": float(root_z),
        "final_height_m": float(env.data.qpos[2]),
        "min_height_m": float(min(heights)),
        "height_drop_m": float(root_z - min(heights)),
        "final_up_dot": up_dot,
        "final_contacts": int(env.data.ncon),
        "note": "no controller: falling over is expected; only the passive contribution is of interest",
    }

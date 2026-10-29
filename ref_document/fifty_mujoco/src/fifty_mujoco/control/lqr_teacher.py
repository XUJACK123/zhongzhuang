"""LQR teacher on the *current* MuJoCo model (doc sections 11.2 - 11.6).

What this is, and what it is not
--------------------------------
It is the **path B prototype**: an LQR for the wheeled-inverted-pendulum
channel, derived from the model's own mass/geometry, plus the leg-length and
leg-angle loops the reference implementation also keeps outside its LQR.  The
four virtual outputs are mapped to the six joint torques through the leg
Jacobian of the actual model and then through the inverse of the mixed PD, so
the teacher speaks exactly the training action contract.

It is **not** a validated teacher.  Nothing here proves the robot stands;
``evaluation/lqr_suite.py`` measures it and the gate in doc 11.6 stays open
until it passes.  In particular:

* the LQR is designed on the reduced planar model, not on the full contact
  dynamics (that is a deliberate scope limit: a linearisation of the full
  model around a contact equilibrium is the next step, not this one),
* the leg length/angle loops are PD with gravity feed-forward, and are
  documented as such rather than called "LQR",
* the reference HKU gains are *not* imported: their mass, geometry, state
  order and 2 ms period belong to another robot.

State feedback uses the simulator's true state.  A student cannot see it, so
the teacher is only a label source (doc 11.4): the actor sees 25/125 and the
encoder learns velocity separately.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Sequence

import mujoco
import numpy as np

from ..config import Config
from ..contracts import names
from .gas_spring import load_springs, reference_force
from .mixed_pd import PdGains

#: Gains that ship with the prototype.  Every one is ``provisional``: they come
#: from the reduced model plus a hand pass in simulation, not from a measured
#: robot, and they must be re-derived for a calibrated model.
DEFAULT_GAINS: dict[str, float] = {
    # wheeled pendulum LQR: Q weights on (v error, pitch, pitch rate)
    "q_velocity": 8.0,
    "q_pitch": 40.0,
    "q_pitch_rate": 2.0,
    "r_wheel_torque": 1.0,
    # leg length loop (N/m, N.s/m)
    "leg_length_kp": 2500.0,
    "leg_length_kd": 120.0,
    # leg angle loop (N.m/rad, N.m.s/rad)
    "leg_angle_kp": 45.0,
    "leg_angle_kd": 3.0,
    "leg_angle_from_pitch": 1.0,
    # safety envelope
    "max_wheel_torque_nm": 5.0,
    "max_leg_torque_nm": 30.0,
    "max_leg_force_n": 400.0,
}


def solve_discrete_lqr(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray,
                       iterations: int = 500, tol: float = 1e-10) -> np.ndarray:
    """Discrete Riccati iteration ``P = Q + A'PA - A'PB(R + B'PB)^-1 B'PA``.

    Iterating the Riccati recursion is enough for the small, stabilisable
    systems used here and avoids a SciPy dependency.  Divergence is reported,
    never silently masked: an unstabilisable pair returns ``inf``.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    p = np.asarray(q, dtype=float).copy()
    for _ in range(iterations):
        bp = b.T @ p
        gain = np.linalg.solve(r + bp @ b, bp @ a)
        p_next = q + a.T @ p @ a - a.T @ p @ b @ gain
        if not np.all(np.isfinite(p_next)):
            return np.full_like(gain, np.inf)
        if np.max(np.abs(p_next - p)) < tol:
            p = p_next
            break
        p = p_next
    bp = b.T @ p
    return np.linalg.solve(r + bp @ b, bp @ a)


@dataclass(frozen=True)
class WheelLegParameters:
    """Reduced planar parameters taken from the MJCF, with their provenance."""

    total_mass_kg: float
    wheel_radius_m: float
    wheel_inertia_kg_m2: float
    com_above_axle_m: float
    leg_length_m: float
    source: str = "mjcf_equivalent_serial_model"

    def as_dict(self) -> dict[str, Any]:
        return {
            "total_mass_kg": self.total_mass_kg,
            "wheel_radius_m": self.wheel_radius_m,
            "wheel_inertia_kg_m2": self.wheel_inertia_kg_m2,
            "com_above_axle_m": self.com_above_axle_m,
            "leg_length_m": self.leg_length_m,
            "source": self.source,
            "status": "provisional",
        }


def wheel_pendulum_matrices(params: WheelLegParameters, gravity: float = 9.81) -> tuple[np.ndarray, np.ndarray]:
    """Continuous A/B of the wheeled inverted pendulum, ``x = [v, pitch, dpitch]``.

    Standard planar model (body of mass ``m`` with its centre of mass ``L``
    above the axle, wheel radius ``r``, inertia ``Iw`` about the axle)::

        (m + Iw/r^2) xdd + m L cos(p) pdd - m L p_dot^2 sin(p) = tau / r
        m L cos(p) xdd + m L^2 pdd - m g L sin(p) = -tau

    The global position is dropped on purpose (doc 11.4): the student has no
    global pose, so the teacher tracks speed, not absolute position.  The body
    is treated as a point mass at the centre of mass, which is a documented
    simplification of this prototype and one of the reasons its gains are
    ``provisional``.
    """
    m = params.total_mass_kg
    r = params.wheel_radius_m
    iw = params.wheel_inertia_kg_m2
    lever = max(params.com_above_axle_m, 0.02)
    alpha = m * lever
    i_theta = m * lever**2
    a11 = m + iw / r**2
    det = a11 * i_theta - alpha * alpha
    if det <= 1e-9:
        raise ValueError("degenerate wheel-pendulum parameters")
    a = np.zeros((3, 3))
    a[0, 1] = -alpha * m * gravity * lever / det
    a[1, 2] = 1.0
    a[2, 1] = a11 * m * gravity * lever / det
    b = np.zeros((3, 1))
    b[0, 0] = (i_theta / r + alpha) / det
    b[2, 0] = (-a11 - alpha / r) / det
    return a, b


@dataclass
class LegGeometry:
    """Per-side leg geometry at the current state."""

    length_m: float
    length_rate_m_s: float
    angle_rad: float
    angle_rate_rad_s: float
    jacobian: np.ndarray  # (2, 2): rows d(length)/dq, d(angle)/dq


class LqrTeacher:
    """Four virtual outputs -> six joint torques -> six actions.

    The interface is the same as a policy (``__call__(obs, history)``) so the
    acceptance evaluator can run it unchanged; ``bind`` gives it the simulator
    state it is allowed to use as a teacher.
    """

    def __init__(
        self,
        config: Config,
        gains: dict[str, float] | None = None,
        *,
        pd: PdGains | None = None,
        height_offset_m: float = 0.0,
    ):
        self.config = config
        self.gains = dict(DEFAULT_GAINS)
        if gains:
            self.gains.update({key: float(value) for key, value in gains.items()})
        self.pd = pd or PdGains(
            leg_kp=float(config.robot["control"]["leg_kp"]),
            leg_kd=float(config.robot["control"]["leg_kd"]),
            wheel_kp=float(config.robot["control"]["wheel_kp"]),
            wheel_kd=float(config.robot["control"]["wheel_kd"]),
            torque_limit_nm=config.effort_limit("rf0_Joint"),
            wheel_torque_limit_nm=config.effort_limit("r_wheel_Joint"),
            leg_position_scale=float(config.robot["control"]["leg_position_scale"]),
            wheel_velocity_scale=float(config.robot["control"]["wheel_velocity_scale"]),
        )
        self.height_offset_m = float(height_offset_m)
        self.env = None
        self.default_angles = np.array(
            [config.default_angles[name] for name in names.MODEL_JOINT_ORDER], dtype=float
        )
        self.springs = load_springs(config) if config.gas_spring_configured else []
        self.reference_forces = np.array(
            [reference_force(spring) for spring in self.springs], dtype=float
        )
        self.params: WheelLegParameters | None = None
        self.k_wheel: np.ndarray | None = None
        self.last_action = np.zeros(names.NUM_ACTIONS)
        self.last_joint_torque = np.zeros(names.NUM_ACTIONS)
        self.saturation = np.zeros(names.NUM_ACTIONS, dtype=bool)
        self.valid = True
        self._neg_length = None
        self._length_target = None

    # ------------------------------------------------------------------ setup
    def bind(self, env) -> "LqrTeacher":
        """Attach to a ``FiftyVecEnv`` and derive the reduced-model parameters."""
        self.env = env
        self.params = self._measure_parameters(env)
        a, b = wheel_pendulum_matrices(self.params)
        dt = env.dt
        discrete_a = np.eye(3) + a * dt
        discrete_b = b * dt
        q = np.diag([self.gains["q_velocity"], self.gains["q_pitch"], self.gains["q_pitch_rate"]])
        r = np.array([[self.gains["r_wheel_torque"]]])
        self.k_wheel = solve_discrete_lqr(discrete_a, discrete_b, q, r)
        # zero the leg angle against the spawn pose: the raw value is ~+90 deg
        # (the hip sits above the wheel), so comparing it to 0 would be a
        # permanent -1.5 rad error and saturate the hip every step.
        self.spawn_leg_angle = {
            side: self.leg_geometry(env, 0, side).angle_rad for side in ("r", "l")
        }
        self._length_target = self._nominal_leg_length(env)
        self.reset()
        return self

    def reset(self) -> None:
        """Clear the integrators/memory.  This teacher has none, but the
        interface is part of the contract: a stateful teacher must expose it."""
        self.last_action = np.zeros(names.NUM_ACTIONS)
        self.last_joint_torque = np.zeros(names.NUM_ACTIONS)
        self.saturation = np.zeros(names.NUM_ACTIONS, dtype=bool)
        self.valid = True

    def _measure_parameters(self, env) -> WheelLegParameters:
        """Reduced parameters from the model itself (no inherited constants)."""
        model = env.model
        data = env.data[0]
        single = env.single
        total_mass = float(np.sum(model.body_mass[1:]))
        wheel_ids = (single.body_ids["right_wheel"], single.body_ids["left_wheel"])
        wheel_id = wheel_ids[0]
        wheel_geoms = single.wheel_geom_ids["right_wheel"]
        wheel_radius = float(max(model.geom_size[geom, 0] for geom in wheel_geoms))
        whirl = single.dof_adr["r_wheel_Joint"]
        mujoco.mj_forward(model, data)
        mass_matrix = np.zeros((model.nv, model.nv))
        mujoco.mj_fullM(model, data, mass_matrix)
        wheel_inertia = float(mass_matrix[whirl, whirl])
        axle_z = float(data.xpos[wheel_id][2])
        mass_sum = 0.0
        weighted_z = 0.0
        for body_id in range(1, model.nbody):
            if body_id in wheel_ids:
                continue
            mass = float(model.body_mass[body_id])
            mass_sum += mass
            weighted_z += mass * float(data.xipos[body_id][2])
        com_above_axle = weighted_z / max(mass_sum, 1e-9) - axle_z
        return WheelLegParameters(
            total_mass_kg=total_mass,
            wheel_radius_m=wheel_radius,
            wheel_inertia_kg_m2=wheel_inertia,
            com_above_axle_m=float(com_above_axle),
            leg_length_m=self._nominal_leg_length(env),
        )

    def _nominal_leg_length(self, env) -> float:
        return self.leg_geometry(env, 0, "r", data=env.data[0]).length_m

    # -------------------------------------------------------------- geometry
    def leg_geometry(self, env, index: int, side: str, data=None) -> LegGeometry:
        """Leg length / angle and their Jacobian for one side, from MuJoCo."""
        model = env.model
        data = env.data[index] if data is None else data
        single = env.single
        hip_name = "rf0_Joint" if side == "r" else "lf0_Joint"
        wheel_body = "right_wheel" if side == "r" else "left_wheel"
        q_names = ("rf0_Joint", "rf1_Joint") if side == "r" else ("lf0_Joint", "lf1_Joint")
        qpos = np.array([data.qpos[single.qpos_adr[name]] for name in q_names])
        qvel = np.array([data.qvel[single.dof_adr[name]] for name in q_names])

        def measure(angles: np.ndarray) -> tuple[float, float]:
            saved = np.array([data.qpos[single.qpos_adr[name]] for name in q_names])
            for name, value in zip(q_names, angles, strict=True):
                data.qpos[single.qpos_adr[name]] = value
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            hip = np.asarray(data.xanchor[single.joint_ids[hip_name]], dtype=float)
            wheel = np.asarray(data.xpos[single.body_ids[wheel_body]], dtype=float)
            quat = np.asarray(data.qpos[3:7], dtype=float)
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, quat / np.linalg.norm(quat))
            rotation = rotation.reshape(3, 3)
            offset = rotation.T @ (wheel - hip)
            for name, value in zip(q_names, saved, strict=True):
                data.qpos[single.qpos_adr[name]] = value
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            return float(np.linalg.norm(offset)), float(np.arctan2(-offset[2], offset[0]))

        length, angle = measure(qpos)
        step = 1e-4
        jac = np.zeros((2, 2))
        for column in range(2):
            perturbed = qpos.copy()
            perturbed[column] += step
            length_p, angle_p = measure(perturbed)
            jac[0, column] = (length_p - length) / step
            jac[1, column] = (angle_p - angle) / step
        length_rate = float(jac[0] @ qvel)
        angle_rate = float(jac[1] @ qvel)
        return LegGeometry(length, length_rate, angle, angle_rate, jac)

    # ------------------------------------------------------------- controller
    def virtual_outputs(self, env, index: int) -> dict[str, float]:
        """The four reference-style outputs plus the diagnostics behind them."""
        data = env.data[index]
        rotation = env._base_rotation(index)
        gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        pitch = float(np.arcsin(np.clip(gravity[0], -1.0, 1.0)))
        _ang_vel, lin_vel = env._base_velocity(index)
        command = env.commands.values[index]
        mass = self.params.total_mass_kg
        weight_per_leg = mass * 9.81 * 0.5
        results: dict[str, float] = {"pitch_rad": pitch, "vx": float(lin_vel[0])}
        state = np.array([lin_vel[0] - command[0], pitch, _ang_vel[1]])
        wheel_torque = float(-(self.k_wheel @ state)[0])
        wheel_torque = float(np.clip(wheel_torque, -self.gains["max_wheel_torque_nm"], self.gains["max_wheel_torque_nm"]))
        results["wheel_torque_nm"] = wheel_torque
        for side in ("r", "l"):
            geometry = self.leg_geometry(env, index, side)
            length_error = self._length_target - geometry.length_m
            # The leg angle must stay vertical *in the world*: the body pitches
            # by ``pitch``, so the body-frame target is ``-pitch``.
            angle_rel = geometry.angle_rad - self.spawn_leg_angle[side]
            angle_error = angle_rel + self.gains["leg_angle_from_pitch"] * pitch
            force = weight_per_leg + self.gains["leg_length_kp"] * length_error - self.gains["leg_length_kd"] * geometry.length_rate_m_s
            force = float(np.clip(force, -self.gains["max_leg_force_n"], self.gains["max_leg_force_n"]))
            torque = -self.gains["leg_angle_kp"] * angle_error - self.gains["leg_angle_kd"] * geometry.angle_rate_rad_s
            torque = float(np.clip(torque, -self.gains["max_leg_torque_nm"], self.gains["max_leg_torque_nm"]))
            results[f"{side}_leg_length_m"] = geometry.length_m
            results[f"{side}_leg_angle_rad"] = angle_rel
            results[f"{side}_leg_force_n"] = force
            results[f"{side}_leg_torque_nm"] = torque
            results[f"{side}_jacobian"] = geometry.jacobian
        return results

    def joint_targets(self, env, index: int) -> np.ndarray:
        """Six joint torques requested by the teacher (doc 11.5)."""
        outputs = self.virtual_outputs(env, index)
        torque = np.zeros(6)
        for side, joints in (("r", (0, 1)), ("l", (3, 4))):
            jac = outputs[f"{side}_jacobian"]
            virtual = np.array([outputs[f"{side}_leg_force_n"], outputs[f"{side}_leg_torque_nm"]])
            torque[list(joints)] = jac.T @ virtual
        wheel_torque = outputs["wheel_torque_nm"]
        torque[list(names.POLICY_WHEEL_INDICES)] = wheel_torque
        return torque

    def torque_to_action(self, torque: np.ndarray, joint_position: np.ndarray, joint_velocity: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Exact inverse of the mixed PD for ``torque_scale == 1`` (doc 11.5).

        Returns ``(action, saturated_mask)``.  A saturated channel is flagged,
        never silently clipped: demonstration labels built from unattainable
        torques are invalid training data.
        """
        torque = np.asarray(torque, dtype=float)
        action = np.zeros(6)
        limits = self.pd.limits()
        saturated = np.abs(torque) > limits + 1e-9
        clipped = np.clip(torque, -limits, limits)
        for index in names.POLICY_LEG_INDICES:
            action[index] = (
                joint_position[index]
                - self.default_angles[index]
                + (clipped[index] + self.pd.leg_kd * joint_velocity[index]) / self.pd.leg_kp
            ) / self.pd.leg_position_scale
        for index in names.POLICY_WHEEL_INDICES:
            action[index] = (
                joint_velocity[index] + clipped[index] / self.pd.wheel_kd
            ) / self.pd.wheel_velocity_scale
        return action, saturated

    # --------------------------------------------------------------- policy
    def __call__(self, obs: np.ndarray, history: np.ndarray | None = None) -> np.ndarray:
        """Evaluate every bound environment; ``obs``/``history`` are ignored.

        Using the simulator state instead of ``obs`` is what makes this a
        teacher (doc 11.4); it is never exported as the deployed policy.
        """
        env = self.env
        if env is None:
            raise RuntimeError("LqrTeacher.bind(env) must be called before use")
        n = obs.shape[0]
        actions = np.zeros((n, names.NUM_ACTIONS))
        for index in range(n):
            position, velocity = env._joint_state(index)
            torque = self.joint_targets(env, index)
            action, saturated = self.torque_to_action(torque, position, velocity)
            actions[index] = action
            self.saturation = saturated
            self.last_action = action
            self.last_joint_torque = torque
        return actions

"""Teacher-action adapters (doc section 11.5).

The teacher lives in a different space than the policy: virtual leg force and
leg-angle torque on one side, wheel torque on the other.  Two conversions are
needed and both are easy to get subtly wrong, so they live here with explicit
signatures and a round-trip test:

``torques_to_action``
    the exact inverse of the mixed PD for the *current* contract
    (``torque_scale == 1``, no spring feed-forward, zero leg velocity target,
    wheel ``Kp = 0``)::

        leg   tau = Kp (q_default + s_leg a - q) - Kd qdot
        wheel tau = Kd (s_wheel a - qdot)

    A torque outside the motor limit is **flagged**, not silently clipped:
    labels built from unattainable torques are not valid demonstrations.

``action_to_torques``
    the forward pass, used by the round-trip test and by the saturation audit.
    It only guarantees equality at the sampled instant: an action is held for
    two physics sub-steps, so the second sub-step torque follows the state, and
    no report may claim exact equivalence with a 500 Hz torque LQR.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import Config
from ..contracts import names
from .gas_spring import load_springs
from .mixed_pd import PdGains, compute_torques, decode_action


@dataclass
class AdapterResult:
    action: np.ndarray
    saturated: np.ndarray
    requested_torque: np.ndarray
    applied_torque: np.ndarray

    @property
    def valid(self) -> bool:
        return not bool(np.any(self.saturated))


def default_gains(config: Config) -> PdGains:
    control = config.robot["control"]
    return PdGains(
        leg_kp=float(control["leg_kp"]),
        leg_kd=float(control["leg_kd"]),
        wheel_kp=float(control["wheel_kp"]),
        wheel_kd=float(control["wheel_kd"]),
        torque_limit_nm=config.effort_limit("rf0_Joint"),
        wheel_torque_limit_nm=config.effort_limit("r_wheel_Joint"),
        leg_position_scale=float(control["leg_position_scale"]),
        wheel_velocity_scale=float(control["wheel_velocity_scale"]),
    )


def torques_to_action(
    torque: np.ndarray,
    joint_position: np.ndarray,
    joint_velocity: np.ndarray,
    config: Config,
    gains: PdGains | None = None,
) -> AdapterResult:
    gains = gains or default_gains(config)
    if gains.torque_scale != 1.0:
        raise NotImplementedError("the inverse PD assumes torque_scale == 1")
    default = np.array([config.default_angles[name] for name in names.MODEL_JOINT_ORDER], dtype=float)
    requested = np.asarray(torque, dtype=float).copy()
    limits = gains.limits()
    saturated = np.abs(requested) > limits + 1e-9
    clipped = np.clip(requested, -limits, limits)
    action = np.zeros(6)
    for index in names.POLICY_LEG_INDICES:
        action[index] = (
            joint_position[index] - default[index] + (clipped[index] + gains.leg_kd * joint_velocity[index]) / gains.leg_kp
        ) / gains.leg_position_scale
    for index in names.POLICY_WHEEL_INDICES:
        action[index] = (joint_velocity[index] + clipped[index] / gains.wheel_kd) / gains.wheel_velocity_scale
    applied = compute_torques(
        decode_action(action, config.default_angles, gains),
        np.asarray(joint_position, dtype=float),
        np.asarray(joint_velocity, dtype=float),
        gains,
    )
    return AdapterResult(action=action, saturated=saturated, requested_torque=requested, applied_torque=applied)


def action_to_torques(
    action: np.ndarray,
    joint_position: np.ndarray,
    joint_velocity: np.ndarray,
    config: Config,
    gains: PdGains | None = None,
) -> np.ndarray:
    gains = gains or default_gains(config)
    return compute_torques(
        decode_action(action, config.default_angles, gains),
        np.asarray(joint_position, dtype=float),
        np.asarray(joint_velocity, dtype=float),
        gains,
    )


def spring_feedforward_torque(config: Config, env, index: int) -> np.ndarray:
    """Generalized torque already delivered by the gas springs.

    A teacher must not ask the motors for work the spring already does (doc
    11.5 item 1).  This returns the six generalized joint torques the passive
    springs contribute in the current state.
    """
    if not config.gas_spring_configured:
        return np.zeros(6)
    from types import SimpleNamespace

    from .gas_spring import spring_force, spring_geometry

    view = SimpleNamespace(model=env.model, data=env.data[index], xml_path=None)
    torque = np.zeros(6)
    for spring in load_springs(config):
        geometry = spring_geometry(view, spring)
        force = spring_force(spring, geometry["length_m"], geometry["length_rate_m_s"])
        torque += geometry["jacobian"][6:] * force
    return torque

"""Canonical naming and ordering contracts.

Two orders must never be conflated:

* **model order**  - the order of joints in the MJCF / URDF tree
                     (``rf0, rf1, r_wheel, lf0, lf1, l_wheel``)
* **policy order** - the order of the six action channels produced by the
                     network (currently identical, but kept as a separate
                     contract on purpose; the teacher ONNX may differ).

Nothing in this package indexes ``qpos[7:]`` or ``qvel[6:]`` directly.
"""

from __future__ import annotations

from dataclasses import dataclass

MODEL_JOINT_ORDER: tuple[str, ...] = (
    "rf0_Joint",
    "rf1_Joint",
    "r_wheel_Joint",
    "lf0_Joint",
    "lf1_Joint",
    "l_wheel_Joint",
)

POLICY_JOINT_ORDER: tuple[str, ...] = MODEL_JOINT_ORDER

LEG_JOINTS: tuple[str, ...] = ("rf0_Joint", "rf1_Joint", "lf0_Joint", "lf1_Joint")
WHEEL_JOINTS: tuple[str, ...] = ("r_wheel_Joint", "l_wheel_Joint")

#: Indices inside the six-vector policy action, in model order.
POLICY_LEG_INDICES: tuple[int, ...] = (0, 1, 3, 4)
POLICY_WHEEL_INDICES: tuple[int, ...] = (2, 5)

#: Dimensions of the Fudan interface that must stay untouched.
NUM_ACTIONS = 6
ACTOR_OBS_DIM = 25
HISTORY_LENGTH = 5
HISTORY_DIM = ACTOR_OBS_DIM * HISTORY_LENGTH  # 125
BASE_PRIVILEGED_DIM = 141
ENCODER_LATENT_DIM = 3

#: Gas-spring privileged features: 4 per spring, 2 springs -> 149 + 3 = 152.
GAS_SPRING_FEATURES_PER_SPRING = 4

LINK_NAMES: tuple[str, ...] = (
    "base_link",
    "r0_link",
    "r1_link",
    "right_wheel",
    "l0_link",
    "l1_link",
    "left_wheel",
)


@dataclass(frozen=True)
class ControlBaseline:
    """Action decoding and PD gains for the target robot (migration baseline)."""

    leg_position_scale: float = 0.5
    wheel_velocity_scale: float = 10.0
    leg_kp: float = 20.0
    leg_kd: float = 1.0
    wheel_kp: float = 0.0
    wheel_kd: float = 0.2
    torque_limit_nm: float = 30.0


@dataclass(frozen=True)
class TeacherBaseline:
    """Historical Fudan configuration, kept separately and never merged."""

    leg_kp: float = 15.0
    leg_kd: float = 1.0
    wheel_kp: float = 0.0
    wheel_kd: float = 0.1
    forward_velocity_scale: float = 3.0


def policy_index(joint_name: str) -> int:
    return POLICY_JOINT_ORDER.index(joint_name)


def privileged_dim(spring_count: int, latent_dim: int = ENCODER_LATENT_DIM) -> tuple[int, int]:
    """Return ``(privileged_obs_dim, critic_input_dim)`` for a spring count."""
    privileged = BASE_PRIVILEGED_DIM + GAS_SPRING_FEATURES_PER_SPRING * spring_count
    return privileged, privileged + latent_dim

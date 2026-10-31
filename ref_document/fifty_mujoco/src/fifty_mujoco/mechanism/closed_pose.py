"""The closed-chain joint values that reproduce a *link* pose.

The planar mechanism works in absolute link angles (``thigh_rad``, ``crank_rad``,
``shin_rad``); MuJoCo works in hinge values measured from whatever the body's
pose happens to be at ``qpos = 0``.  Those are different quantities and mixing
them is the classic way to get a pose that looks plausible and is wrong.  This
module is the one place that converts between them:

    qpos[hinge] = sign * wrap(abs_angle_at_target - abs_angle_at_zero)

Every angle comes from geometry (node positions), never from a hand-typed table,
so ``tests/test_closed_pose.py`` can recompute it.

Why a keyframe instead of a solver
----------------------------------
MuJoCo can pull the passive joints into place with its equality constraints, but
only if it starts close enough and on the right branch.  Rather than hope, the
values are computed here and written as a ``<keyframe>``: the reset then *starts*
at a closed, non-singular pose instead of converging to one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

from .loader import Mechanism
from .transmission import ThighCrankTransmission

Vec2 = tuple[float, float]

#: ``(body, proximal node, nodes it carries)`` in the confirmed
#: ``link_a_on_thigh`` chain.  The order matters: the first node fixes the body's
#: absolute angle.
BODIES: tuple[tuple[str, str, tuple[str, ...]], ...] = (
    ("thigh", "HIP", ("KNEE", "LINK_PIN_A")),
    ("shin", "KNEE", ("END_PIN_A", "WHEEL")),
    ("wheel", "WHEEL", ()),
    ("crank", "HIP", ("CRANK_PIN",)),
    ("push_rod", "CRANK_PIN", ("ROD_PIN",)),
    ("ternary", "ROD_PIN", ("LINK_PIN_A", "LINK_PIN_C")),
    ("end_link", "LINK_PIN_C", ("END_PIN_A",)),
)

#: Bodies whose reported angle uses the **planar** convention (the same pair the
#: planar model and ``bridge.py`` use) instead of the first carried node.  The
#: hinge value is identical either way - the constant plate angle cancels between
#: the target and the zero pose - but these are the numbers a reader checks
#: against ``docs/JOINT_CONVENTIONS.md``.
PLANAR_PAIRS: Mapping[str, tuple[str, str]] = {
    "thigh": ("HIP", "KNEE"),
    "shin": ("KNEE", "WHEEL"),
    "crank": ("HIP", "CRANK_PIN"),
}

#: Body -> hinge name, with ``{key}`` replaced by ``r`` / ``l``.
JOINT_NAMES: Mapping[str, str] = {
    "thigh": "{key}f0_Joint",
    "crank": "{key}f1_Joint",
    "wheel": "{key}_wheel_Joint",
    "shin": "{key}_knee_Joint",
    "push_rod": "{key}_pushrod_Joint",
    "ternary": "{key}_ternary_Joint",
    "end_link": "{key}_endlink_Joint",
}

#: Bodies whose hinge is driven; the rest are determined by the two loops.
DRIVEN = ("thigh", "crank", "wheel")

#: Kinematic parent of every body in the generated model.  A hinge rotates the
#: body *relative to its parent*, so the conversion needs this tree - the first
#: version subtracted each body's absolute angle directly, which is only right
#: for the two bodies whose parent is the trunk.
PARENT: Mapping[str, str] = {
    "thigh": "trunk",
    "shin": "thigh",
    "wheel": "shin",
    "crank": "trunk",
    "push_rod": "crank",
    "ternary": "push_rod",
    "end_link": "ternary",
}

#: Hinge turn axis sign per side, in the base frame.
AXIS_SIGN = {"right": 1.0, "left": -1.0}

#: Emission (and therefore ``qpos``) order of the bodies, ``(body, parent)``.
TREE: tuple[tuple[str, str], ...] = tuple(
    (body, PARENT[body])
    for body in ("thigh", "shin", "wheel", "crank", "push_rod", "ternary", "end_link")
)

SIDES = (("right", "r"), ("left", "l"))


def _angle(origin: Vec2, target: Vec2) -> float:
    return math.atan2(target[1] - origin[1], target[0] - origin[0])


def wrap(angle: float) -> float:
    """Wrap into ``(-pi, pi]``."""
    wrapped = math.remainder(angle, 2.0 * math.pi)
    return math.pi if wrapped == -math.pi else wrapped


def body_angles(nodes: Mapping[str, Vec2]) -> dict[str, float]:
    """Absolute planar angle of every body, from the node positions."""
    angles: dict[str, float] = {}
    for body, proximal, carried in BODIES:
        if not carried:
            continue
        pair = PLANAR_PAIRS.get(body)
        if pair is not None:
            angles[body] = _angle(nodes[pair[0]], nodes[pair[1]])
        else:
            angles[body] = _angle(nodes[proximal], nodes[carried[0]])
    return angles


@dataclass(frozen=True)
class ClosedPose:
    """One fully specified closed-chain configuration."""

    #: body -> absolute planar angle (radians, ``atan2(z, y)``)
    angles: Mapping[str, float]
    #: body -> hinge value (radians), sign and zero already applied
    qpos: Mapping[str, float]
    #: hip-to-wheel chord (metres)
    chord_m: float

    def joint_values(self, side: str) -> dict[str, float]:
        """Hinge values for one side.

        ``qpos`` is stored in the right-hand (canonical) convention; a left leg
        hinge turns about ``-Y`` so its value is the negation, while the two wheels
        share ``+Y`` and keep the value as-is.
        """
        key = dict(SIDES)[side]
        sign = AXIS_SIGN[side]
        return {
            JOINT_NAMES[body].format(key=key): (
                self.qpos[body] if body == "wheel" else sign * self.qpos[body]
            )
            for body in JOINT_NAMES
            if body in self.qpos
        }


def _qpos_from(target: Mapping[str, float], zero: Mapping[str, float]) -> dict[str, float]:
    """Right-hand (+Y) hinge values for a pose, as the generated model stores them.

    Two things the first version got wrong, both caught by compiling the model:

    * a hinge is a **relative** rotation, so a child's value is the difference
      of its angle change and its parent's;
    * a positive rotation about +Y **decreases** the planar angle ``atan2(z, y)``
      (the model's body frames are unrotated at ``qpos = 0``, and the CAD plane
      maps ``Y -> X``, ``Z -> Z``), so the value carries a minus sign.

    Hence ``q = -((target - zero) - (target_parent - zero_parent))``.  With this
    sign the two loop residuals are ~1e-12 m and the gas-spring tendon measures
    227.760939 mm; with the old absolute-difference formula the thigh alone is
    off by 2x1.7385 rad and the loops miss by 0.24 m / 0.44 m.
    """
    values: dict[str, float] = {"wheel": 0.0}
    for body in target:
        if body == "wheel":
            continue
        parent = PARENT.get(body, "trunk")
        delta = wrap(target[body] - zero[body])
        parent_delta = wrap(target[parent] - zero[parent]) if parent in target else 0.0
        values[body] = wrap(-(delta - parent_delta))
    return values


def assembly_pose(mechanism: Mechanism) -> ClosedPose:
    """``qpos = 0``: every angle is whatever the CAD assembly draws."""
    nodes = {name: node.yz_mm for name, node in mechanism.nodes.items()}
    angles = body_angles(nodes)
    chord = math.dist(nodes["HIP"], nodes["WHEEL"]) / 1000.0
    return ClosedPose(
        angles=angles,
        qpos={**{body: 0.0 for body in angles}, "wheel": 0.0},
        chord_m=chord,
    )


def pose_at(
    mechanism: Mechanism,
    transmission: ThighCrankTransmission,
    thigh_rad: float,
    crank_rad: float,
    side: str = "right",
) -> ClosedPose:
    """Closed-chain configuration for a driver pair, as hinge values."""
    del side  # the side only affects the sign, applied by ``joint_values``
    zero = assembly_pose(mechanism).angles
    nodes = transmission.forward(thigh_rad, crank_rad)
    angles = body_angles(nodes)
    return ClosedPose(
        angles=angles,
        qpos=_qpos_from(angles, zero),
        chord_m=math.dist(nodes["HIP"], nodes["WHEEL"]),
    )


def reference_pose(
    mechanism: Mechanism | None = None, side: str = "right"
) -> ClosedPose:
    """The standing reference pose as closed-chain hinge values.

    The angles are read from ``configs/joint_conventions.yaml`` rather than
    retyped, and they are the planar **absolute** angles of
    ``reference_pose.solution`` - not hinge values.
    """
    from ..config import load_yaml
    from ..paths import configs_dir
    from .loader import load_mechanism

    mech = mechanism if mechanism is not None else load_mechanism()
    solution = load_yaml(configs_dir() / "joint_conventions.yaml")["reference_pose"]["solution"]
    return reference_pose_from_angles(
        mech, float(solution["thigh_rad"]), float(solution["crank_rad"]), side
    )


def reference_pose_from_angles(
    mechanism: Mechanism, thigh_rad: float, crank_rad: float, side: str = "right"
) -> ClosedPose:
    """As :func:`reference_pose` but with the angles supplied by the caller."""
    transmission = ThighCrankTransmission.from_mechanism(mechanism)
    return pose_at(mechanism, transmission, thigh_rad, crank_rad, side)


def serialize(pose: ClosedPose) -> dict[str, object]:
    """JSON-able form, rounded so the reports do not churn on float noise."""
    return {
        "angles_rad": {body: round(value, 12) for body, value in pose.angles.items()},
        "qpos_rad": {body: round(value, 12) for body, value in pose.qpos.items()},
        "chord_m": round(pose.chord_m, 12),
    }

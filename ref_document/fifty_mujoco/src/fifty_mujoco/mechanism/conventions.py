"""Joint signs, zeros, axes and the reference standing pose.

The simulation is allowed to pick its own positive direction and zero
(progress.md §23.4), but the same transform has to be applied to the position,
the velocity, the torque, the Jacobian, the observation and the action. This
module is that transform, and it is *checked* rather than asserted:

* :func:`mjcf_joint_axes` reads the axes out of ``assets/mjcf/fifty_train.xml``
  by composing the body quaternions - standard library only, no MuJoCo, no
  dynamics. It is a static statement about the model file, which is exactly what
  the declaration has to agree with.
* :func:`Conventions.verify` compares the declared rows against that file, the
  policy contract and ``robot.yaml``.
* :func:`reference_pose` recomputes the standing pose from the target chord and
  the confirmed closed chain, so the stored numbers cannot silently drift.

Hardware zeroes and wiring directions are *not* here: they stay a deployment
calibration, and this file says so out loud.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from ..paths import assets_dir, configs_dir

Vec3 = tuple[float, float, float]

#: Tolerance when comparing a declared axis with the one read from the MJCF.
AXIS_TOLERANCE = 1e-9


class JointConventionsError(ValueError):
    """The conventions file contradicts itself or the model."""


@dataclass(frozen=True)
class Channel:
    """One of the six policy channels."""

    name: str
    role: str
    side: str
    axis_base: Vec3
    sign: float
    zero_rad: float
    limit_rad: tuple[float, float] | None
    drives: str

    @property
    def is_left(self) -> bool:
        return self.side == "left"


@dataclass(frozen=True)
class Conventions:
    path: Path
    meta: Mapping[str, Any]
    channels: tuple[Channel, ...]
    reference: Mapping[str, Any]

    # ------------------------------------------------------------- lookups
    def channel(self, name: str) -> Channel:
        for entry in self.channels:
            if entry.name == name:
                return entry
        raise JointConventionsError(f"no channel named {name!r}")

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(entry.name for entry in self.channels)

    def counterpart(self, name: str) -> Channel:
        """The channel on the other side with the same role."""
        entry = self.channel(name)
        for other in self.channels:
            if other.role == entry.role and other.side != entry.side:
                return other
        raise JointConventionsError(f"{name!r} has no counterpart on the other side")

    # ----------------------------------------------------------- transforms
    def to_right_frame(self, name: str, value: float) -> float:
        """Express a left-channel value in the right-hand convention.

        ``q_left = sign * q_right`` therefore ``q_right = value / sign``; with the
        declared signs that is a plain negation on the left leg. Velocity and
        torque use the same sign, which is why this is not a "bias" operation:
        the zero offset must never be added to a rate.
        """
        entry = self.channel(name)
        if entry.sign == 0.0:
            raise JointConventionsError(f"{name}: sign must not be zero")
        return value / entry.sign

    def to_own_frame(self, name: str, right_value: float) -> float:
        """Inverse of :meth:`to_right_frame`."""
        return right_value * self.channel(name).sign

    def mirror_pose(self, values: Sequence[float]) -> tuple[float, ...]:
        """Copy the right-hand channels onto the left ones for a symmetric pose."""
        if len(values) != len(self.channels):
            raise JointConventionsError(
                f"expected {len(self.channels)} channel values, got {len(values)}"
            )
        by_name = dict(zip(self.names, values))
        mirrored = dict(by_name)
        for entry in self.channels:
            if entry.is_left:
                mirrored[entry.name] = self.to_own_frame(
                    entry.name, by_name[self.counterpart(entry.name).name]
                )
        return tuple(mirrored[name] for name in self.names)

    def symmetric(self, values: Sequence[float], tolerance: float = 1e-9) -> bool:
        """True when a pose is left/right symmetric in the declared sense."""
        mirrored = self.mirror_pose(values)
        return all(
            abs(left - right) <= tolerance for left, right in zip(values, mirrored)
        )

    # ------------------------------------------------------------ reference
    def reference_channel_values(self) -> dict[str, float]:
        """Reference pose expressed in the *mechanism* coordinates."""
        solution = self.reference.get("solution") or {}
        return {
            "thigh_rad": float(solution["thigh_rad"]),
            "shin_rad": float(solution["shin_rad"]),
            "crank_rad": float(solution["crank_rad"]),
        }

    # ---------------------------------------------------------- verification
    def verify(
        self,
        axes: Mapping[str, Vec3] | None = None,
        robot: Mapping[str, Any] | None = None,
        contract: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Compare every declared row with the model and the other config files."""
        problems: list[str] = []
        measured = dict(axes) if axes is not None else mjcf_joint_axes()

        for entry in self.channels:
            actual = measured.get(entry.name)
            if actual is None:
                problems.append(f"{entry.name}: the MJCF has no such joint")
                continue
            if not _same_direction(entry.axis_base, actual):
                problems.append(
                    f"{entry.name}: declared axis {_fmt(entry.axis_base)} but the MJCF"
                    f" composes to {_fmt(actual)}"
                )
            if robot is not None:
                default = (robot.get("joints") or {}).get("default_angles_rad") or {}
                if entry.name in default:
                    if abs(float(default[entry.name]) - entry.zero_rad) > 1e-12:
                        problems.append(
                            f"{entry.name}: zero_rad {entry.zero_rad!r} does not match"
                            f" robot.yaml {float(default[entry.name])!r}"
                        )
                limits = (robot.get("joints") or {}).get("limits_rad") or {}
                declared = entry.limit_rad
                from_config = limits.get(entry.name)
                if declared is not None and from_config is not None:
                    if any(
                        abs(float(a) - float(b)) > 1e-9
                        for a, b in zip(declared, from_config)
                    ):
                        problems.append(
                            f"{entry.name}: limit_rad {declared} does not match"
                            f" robot.yaml {tuple(from_config)}"
                        )
            if contract is not None:
                signs = ((contract.get("orders") or {}).get("signs")) or {}
                if entry.name in signs and abs(float(signs[entry.name]) - entry.sign) > 1e-12:
                    problems.append(
                        f"{entry.name}: sign {entry.sign!r} does not match the policy"
                        f" contract {float(signs[entry.name])!r}"
                    )

        return {
            "status": "passed" if not problems else "failed",
            "axes": {
                name: [_round(value) for value in axis] for name, axis in measured.items()
            },
            "problems": problems,
        }


# ---------------------------------------------------------------- MJCF axes
def _quat_rotate(quat: tuple[float, float, float, float], vector: Vec3) -> Vec3:
    w, x, y, z = quat
    # rotate v by q: v' = v + 2 * cross(q_vec, cross(q_vec, v) + w * v)
    cx = y * vector[2] - z * vector[1]
    cy = z * vector[0] - x * vector[2]
    cz = x * vector[1] - y * vector[0]
    tx = cx + w * vector[0]
    ty = cy + w * vector[1]
    tz = cz + w * vector[2]
    return (
        vector[0] + 2.0 * (y * tz - z * ty),
        vector[1] + 2.0 * (z * tx - x * tz),
        vector[2] + 2.0 * (x * ty - y * tx),
    )


def _quat_mul(
    first: tuple[float, float, float, float], second: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = first
    w2, x2, y2, z2 = second
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _numbers(text: str | None, count: int, default: Vec3) -> Vec3:
    if not text:
        return default
    values = [float(part) for part in text.replace(",", " ").split()]
    if len(values) < count:
        return default
    return (values[0], values[1], values[2])


@dataclass(frozen=True)
class ChainLink:
    """One body of the MJCF tree, as written in the file.

    Everything except ``axis_base`` is frame-independent: ``offset_parent_m`` is
    the ``body pos`` attribute (parent frame), ``axis_local`` the ``joint axis``
    attribute (own frame).  ``axis_base`` is the local axis rotated by the
    cumulative body quaternion, i.e. the direction the hinge actually turns about
    in the base frame at ``qpos = 0``.
    """

    body: str
    parent: str | None
    joint: str | None
    offset_parent_m: Vec3
    offset_base_m: Vec3
    quat: tuple[float, float, float, float]
    axis_local: Vec3
    axis_base: Vec3
    #: cumulative body quaternion relative to the model root, at ``qpos = 0``
    quat_base: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    @property
    def offset_norm_m(self) -> float:
        return math.sqrt(sum(component * component for component in self.offset_base_m))

    def projected_lengths(self) -> tuple[float, float]:
        """``(planar, axial)`` split of the parent->child offset.

        ``planar`` is the length **perpendicular to the joint axis** - what the
        planar (sagittal) model needs.  ``axial`` is the component along the axis,
        which for this robot is a sideways mounting offset and therefore must not
        be folded into the planar link length.

        The offset has to be projected in the *base* frame, because ``body pos`` is
        written in the parent frame while the hinge turns about ``axis_base``.
        Projecting the raw attribute against the base axis was the error that made
        the planar lengths look like 210.315 / 253.947 mm.
        """
        axis = _normalise(self.axis_base)
        along = sum(component * unit for component, unit in zip(self.offset_base_m, axis))
        planar = math.sqrt(
            max(
                sum(component * component for component in self.offset_base_m)
                - along * along,
                0.0,
            )
        )
        return planar, along


def mjcf_chain(path: str | Path | None = None) -> dict[str, ChainLink]:
    """Every body of ``fifty_train.xml`` with its joint, offset and base axis.

    Pure standard library: this is a static read of the model file, not a
    simulation.  It is the single place that answers "how long is this link
    really", so the model generator, the planar kinematics and the reports cannot
    drift into using different numbers.
    """
    target = Path(path) if path is not None else assets_dir() / "mjcf" / "fifty_train.xml"
    if not target.is_file():
        raise JointConventionsError(f"MJCF not found: {target}")
    root = ElementTree.parse(target).getroot()
    links: dict[str, ChainLink] = {}

    def walk(
        element: ElementTree.Element,
        parent: str | None,
        rotation: tuple[float, float, float, float],
    ) -> None:
        for child in element:
            if child.tag != "body":
                continue
            quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
            raw = child.get("quat")
            if raw:
                parts = [float(part) for part in raw.replace(",", " ").split()]
                if len(parts) == 4:
                    quat = (parts[0], parts[1], parts[2], parts[3])
            combined = _quat_mul(rotation, quat)
            joint_name: str | None = None
            axis_local: Vec3 = (0.0, 0.0, 1.0)
            for item in child:
                if item.tag == "joint" and item.get("name") and item.get("type", "hinge") == "hinge":
                    joint_name = item.get("name")
                    axis_local = _normalise(_numbers(item.get("axis"), 3, (0.0, 0.0, 1.0)))
            name = child.get("name") or f"<unnamed {len(links)}>"
            offset_parent = _numbers(child.get("pos"), 3, (0.0, 0.0, 0.0))
            links[name] = ChainLink(
                body=name,
                parent=parent,
                joint=joint_name,
                offset_parent_m=offset_parent,
                offset_base_m=_quat_rotate(rotation, offset_parent),
                quat=quat,
                axis_local=axis_local,
                axis_base=_normalise(_quat_rotate(combined, axis_local)),
                quat_base=combined,
            )
            walk(child, name, combined)

    for worldbody in root.findall("worldbody"):
        walk(worldbody, None, (1.0, 0.0, 0.0, 0.0))
    return links


def mjcf_body_positions(
    qpos: Mapping[str, float] | None = None, path: str | Path | None = None
) -> dict[str, Vec3]:
    """Base-frame body origins for a hinge configuration, from the MJCF.

    A hinge keeps its body's origin fixed and rotates everything below it, so the
    forward kinematics is just "offset, then rotate about the base-frame axis".
    Standard library only - this exists so the poses can be checked without
    importing MuJoCo.
    """
    links = mjcf_chain(path)
    angles = dict(qpos or {})
    positions: dict[str, Vec3] = {}
    rotations: dict[str | None, tuple[float, float, float, float]] = {
        None: (1.0, 0.0, 0.0, 0.0)
    }

    def visit(parent: str | None) -> None:
        for link in links.values():
            if link.parent != parent:
                continue
            rotation = rotations.get(parent, (1.0, 0.0, 0.0, 0.0))
            offset = _quat_rotate(rotation, link.offset_parent_m)
            origin = positions.get(parent or "", (0.0, 0.0, 0.0))
            positions[link.body] = (
                origin[0] + offset[0],
                origin[1] + offset[1],
                origin[2] + offset[2],
            )
            angle = float(angles.get(link.joint, 0.0)) if link.joint else 0.0
            spin = _axis_quat(link.axis_base, angle) if link.joint else (1.0, 0.0, 0.0, 0.0)
            # the body's own quat has to stay in the chain: `axis_base` was
            # computed from `R_parent * R_quat`, so the runtime rotation is
            # `R_axis(q) * R_parent * R_quat` - dropping the quat here silently
            # misplaces every child body
            rotations[link.body] = _quat_mul(spin, _quat_mul(rotation, link.quat))
            visit(link.body)

    visit(None)
    return positions


def _axis_quat(axis: Vec3, angle: float) -> tuple[float, float, float, float]:
    unit = _normalise(axis)
    half = 0.5 * angle
    sin = math.sin(half)
    return (math.cos(half), unit[0] * sin, unit[1] * sin, unit[2] * sin)


def mjcf_joint_axes(path: str | Path | None = None) -> dict[str, Vec3]:
    """Joint axis of every hinge in the base frame, from the MJCF.

    A joint's ``axis`` is written in its own body frame; the effective axis in the
    base frame is that vector rotated by the *cumulative* body quaternion from
    ``base_link`` down. Reading the file is a static statement about the model -
    no MuJoCo, no simulation, no state.
    """
    target = Path(path) if path is not None else assets_dir() / "mjcf" / "fifty_train.xml"
    if not target.is_file():
        raise JointConventionsError(f"MJCF not found: {target}")
    root = ElementTree.parse(target).getroot()
    axes: dict[str, Vec3] = {}

    def walk(element: ElementTree.Element, rotation: tuple[float, float, float, float]) -> None:
        for child in element:
            if child.tag != "body":
                continue
            quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
            raw = child.get("quat")
            if raw:
                parts = [float(part) for part in raw.replace(",", " ").split()]
                if len(parts) == 4:
                    quat = (parts[0], parts[1], parts[2], parts[3])
            combined = _quat_mul(rotation, quat)
            for item in child:
                if item.tag == "joint" and item.get("name"):
                    axis_local = _numbers(item.get("axis"), 3, (0.0, 0.0, 1.0))
                    axes[item.get("name", "")] = _normalise(
                        _quat_rotate(combined, axis_local)
                    )
            walk(child, combined)

    for worldbody in root.findall("worldbody"):
        walk(worldbody, (1.0, 0.0, 0.0, 0.0))
    return axes


def _normalise(vector: Vec3) -> Vec3:
    length = math.sqrt(sum(component * component for component in vector))
    if length == 0.0:
        return (0.0, 0.0, 0.0)
    return tuple(component / length for component in vector)  # type: ignore[return-value]


def _same_direction(first: Vec3, second: Vec3) -> bool:
    return all(abs(a - b) <= AXIS_TOLERANCE for a, b in zip(_normalise(first), _normalise(second)))


def _fmt(vector: Vec3) -> str:
    return "[" + ", ".join(f"{value:+.1f}" for value in vector) + "]"


def _round(value: float) -> float:
    return round(value, 12) + 0.0


# ------------------------------------------------------------------- loading
def parse_conventions(data: Mapping[str, Any], path: Path) -> Conventions:
    meta = data.get("meta") or {}
    raw_channels = data.get("channels")
    if not isinstance(raw_channels, list) or not raw_channels:
        raise JointConventionsError("channels: expected a non-empty list")
    channels: list[Channel] = []
    for raw in raw_channels:
        if not isinstance(raw, Mapping):
            raise JointConventionsError("channels: every entry must be a mapping")
        missing = [
            key
            for key in ("name", "role", "side", "axis_base", "sign", "zero_rad", "drives")
            if key not in raw
        ]
        if missing:
            raise JointConventionsError(
                f"channels[{raw.get('name')}]: missing {', '.join(missing)}"
            )
        axis = tuple(float(value) for value in raw["axis_base"])
        if len(axis) != 3:
            raise JointConventionsError(f"channels[{raw['name']}].axis_base: need three numbers")
        limits = raw.get("limit_rad")
        channels.append(
            Channel(
                name=str(raw["name"]),
                role=str(raw["role"]),
                side=str(raw["side"]),
                axis_base=axis,  # type: ignore[arg-type]
                sign=float(raw["sign"]),
                zero_rad=float(raw["zero_rad"]),
                limit_rad=(
                    (float(limits[0]), float(limits[1])) if limits else None
                ),
                drives=str(raw["drives"]),
            )
        )
    names = [entry.name for entry in channels]
    if len(set(names)) != len(names):
        raise JointConventionsError("channels: duplicate names")
    for entry in channels:
        if entry.side not in ("left", "right"):
            raise JointConventionsError(f"{entry.name}: side must be left or right")
        if entry.sign not in (1.0, -1.0):
            raise JointConventionsError(f"{entry.name}: sign must be +1 or -1")
    reference = data.get("reference_pose") or {}
    solution = reference.get("solution") or {}
    missing_solution = [
        key
        for key in ("thigh_rad", "shin_rad", "crank_rad")
        if key not in solution
    ]
    if missing_solution:
        raise JointConventionsError(
            f"reference_pose.solution: missing {', '.join(missing_solution)}"
        )
    if not (reference.get("target") or {}).get("chord_m"):
        raise JointConventionsError("reference_pose.target: missing chord_m")
    return Conventions(
        path=path, meta=meta, channels=tuple(channels), reference=reference
    )


def load_conventions(path: str | Path | None = None) -> Conventions:
    target = Path(path) if path is not None else configs_dir() / "joint_conventions.yaml"
    with target.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, Mapping):
        raise JointConventionsError(f"{target}: expected a mapping at the top level")
    return parse_conventions(data, target)


# ------------------------------------------------------------- reference pose
def reference_pose(conventions: Conventions, mechanism, transmission=None):
    """Recompute the stored reference standing pose from the target chord.

    The two-link IK has two elbow solutions; only one of them is a pose the
    *confirmed* closed chain can actually hold at that chord. Both are tried and
    the answer must be unique - if two branches ever close, this raises instead
    of picking one, because the wrong branch is a mirrored knee and no controller
    survives that.
    """
    from .bridge import leg_pose_from_serial
    from .leg import TwoLinkLeg
    from .planar import NoSolutionError
    from .transmission import ThighCrankTransmission

    target = conventions.reference.get("target") or {}
    chord = float(target.get("chord_m", 0.0))
    if chord <= 0.0:
        raise JointConventionsError("reference_pose.target.chord_m must be positive")
    offset = float(target.get("wheel_offset_from_hip_m", 0.0))
    leg = TwoLinkLeg(
        mechanism.length_m("thigh_hip_to_knee"), mechanism.length_m("shin_knee_to_wheel")
    )
    chain = transmission or ThighCrankTransmission.from_mechanism(mechanism)

    solutions = []
    for branch in (0, 1):
        thigh, shin = leg.inverse((0.0, 0.0), (-offset, -chord), branch=branch)
        try:
            pose = leg_pose_from_serial(chain, thigh, shin)
        except NoSolutionError:
            continue
        if abs(pose.chord_m - chord) > 1e-9:
            continue
        if abs(pose.wheel_from_hip[0] - (-offset)) > 1e-9:
            continue
        solutions.append(pose)
    if not solutions:
        raise JointConventionsError(
            "no IK branch of the confirmed chain reaches the reference chord"
            f" {chord:.9f} m"
        )
    if len(solutions) > 1:
        raise JointConventionsError(
            "both IK branches close at the reference chord; the reference pose is"
            " ambiguous and must be chosen explicitly"
        )
    return solutions[0]

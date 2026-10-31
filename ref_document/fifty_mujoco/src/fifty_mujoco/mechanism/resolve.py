"""One set of link lengths, obtained by projection instead of assumption.

There used to be two disagreeing numbers for the leg:

* the planar model measured **210.000 / 250.000 mm** from the STEP assembly;
* the URDF assets, read as 3D distances between body origins, gave
  **210.315 / 253.947 mm**.

The second one was wrong *as a link length*.  The URDF ``body pos`` vector carries
a **sideways mounting offset** - the leg plates sit at different lateral
positions - and that component has to be projected out.  With ``axis`` the hinge
axis and ``d`` the parent->child origin offset,

    planar = || d - (d . axis) axis ||          axial = d . axis

The projection gives **exactly** 210.000 / 250.000 mm on both sides, and the 3D
distances stay 210.315 / 253.947 mm, which is correct *as a 3D distance*: it just
answers a different question.  Using it as the planar length shortened the end
link by 0.357 mm and moved the reference pose by 23.1 mrad; both of those
conclusions are withdrawn here.

So this module does not re-place nodes.  It:

1. reads the model file (``assets/mjcf/fifty_train.xml``, standard library only),
2. projects every leg link perpendicular to its hinge axis,
3. **asserts** the projection equals both the CAD node span and the declared
   authority, so a changed model or measurement fails loudly instead of quietly
   producing a third number,
4. reports the sideways offsets, which are genuinely part of the mounting.

Consequence: ``configs/mechanism.yaml: planar_nodes`` is the single length source
and needs no correction, the end link stays at its CAD 96.600 mm, and the
reference standing pose is unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

from .conventions import ChainLink, mjcf_chain
from .loader import Mechanism

Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]

#: |projected - declared / CAD| that still counts as the same length, in mm.
PLANAR_TOLERANCE_MM = 1e-9

#: The main links, per side: ``(side, thigh body, shin body)``.
SIDE_CHAINS: tuple[tuple[str, str, str], ...] = (
    ("right", "r1_link", "right_wheel"),
    ("left", "l1_link", "left_wheel"),
)


def project_offset(offset_m: Vec3, axis: Vec3) -> dict[str, float]:
    """Split one parent->child offset into ``planar`` and ``axial`` components.

    ``offset_m`` must already be expressed in the frame the axis lives in (the
    base frame for a model file); projecting a parent-frame vector against a
    base-frame axis is exactly the mistake that produced 210.315 mm.
    """
    if not all(math.isfinite(component) for component in (*offset_m, *axis)):
        raise ValueError("offset and axis must be finite (non-finite input)")
    norm = math.sqrt(sum(component * component for component in axis))
    if norm <= 0.0:
        raise ValueError("zero hinge axis: the projection is undefined")
    unit = tuple(component / norm for component in axis)
    along = sum(component * value for component, value in zip(offset_m, unit))
    planar_sq = sum(component * component for component in offset_m) - along * along
    return {
        "planar_length_m": math.sqrt(max(planar_sq, 0.0)),
        "axial_offset_m": along,
    }


@dataclass(frozen=True)
class LinkProjection:
    """One leg link: its 3D offset, its projection, and the sideways part."""

    side: str
    link: str
    joint: str
    body: str
    offset_3d_m: Vec3
    offset_base_m: Vec3
    axis_base: Vec3
    planar_length_m: float
    axial_offset_m: float
    axial_offset_local_m: float = 0.0

    @property
    def origin_distance_m(self) -> float:
        return math.sqrt(sum(component * component for component in self.offset_3d_m))

    def as_dict(self) -> dict[str, object]:
        return {
            "joint": self.joint,
            "body": self.body,
            "planar_length_m": self.planar_length_m,
            "origin_distance_m": self.origin_distance_m,
            "axial_offset_m": self.axial_offset_m,
            "axial_offset_local_m": self.axial_offset_local_m,
            "axis_base": list(self.axis_base),
        }


@dataclass(frozen=True)
class ResolvedNodes:
    """The planar node table plus the evidence that it is the right one."""

    nodes_mm: Mapping[str, Vec2]
    projections: tuple[LinkProjection, ...]
    cad_planar_mm: Mapping[str, float]
    end_link_mm: float

    # ------------------------------------------------------------- lookups
    def _entry(self, side: str, link: str) -> LinkProjection:
        for entry in self.projections:
            if entry.side == side and entry.link == link:
                return entry
        raise KeyError(f"{side}/{link}")

    def planar_mm(self, side: str, link: str) -> float:
        return self._entry(side, link).planar_length_m * 1000.0

    def _common_mm(self, link: str) -> float:
        values = {
            round(entry.planar_length_m * 1000.0, 9)
            for entry in self.projections
            if entry.link == link
        }
        if len(values) != 1:
            raise ValueError(f"the two sides disagree on the {link} length: {sorted(values)}")
        return values.pop()

    @property
    def thigh_mm(self) -> float:
        """Planar thigh length (identical on both sides)."""
        return self._common_mm("thigh")

    @property
    def shin_mm(self) -> float:
        """Planar shin length (identical on both sides)."""
        return self._common_mm("shin")

    @property
    def end_link_cad_mm(self) -> float:
        return math.dist(self.nodes_mm["LINK_PIN_C"], self.nodes_mm["END_PIN_A"])

    #: No node is moved any more, so these stay empty / zero on purpose.
    @property
    def moved_mm(self) -> dict[str, float]:
        return {}

    @property
    def worst_move_mm(self) -> float:
        return 0.0

    def lateral_mm(self, side: str) -> dict[str, float]:
        """Sideways offset of the knee and of the wheel, relative to the hip."""
        thigh = self._entry(side, "thigh")
        shin = self._entry(side, "shin")
        return {
            "knee_from_hip_mm": thigh.axial_offset_m * 1000.0,
            "wheel_from_knee_mm": shin.axial_offset_m * 1000.0,
            "wheel_from_hip_mm": (thigh.axial_offset_m + shin.axial_offset_m) * 1000.0,
        }

    @property
    def worst_planar_error_mm(self) -> float:
        errors = [
            abs(entry.planar_length_m * 1000.0 - self.cad_planar_mm[entry.link])
            for entry in self.projections
        ]
        return max(errors) if errors else 0.0

    def lengths(self) -> dict[str, float]:
        """Every pairwise node distance (millimetres), for the reports."""
        names = list(self.nodes_mm)
        return {
            f"{first}-{second}": math.dist(self.nodes_mm[first], self.nodes_mm[second])
            for index, first in enumerate(names)
            for second in names[index + 1 :]
        }

    # ------------------------------------------------------------- reporting
    @property
    def urdf_projection(self) -> dict[str, object]:
        sides: dict[str, dict[str, object]] = {}
        for side, _, _ in SIDE_CHAINS:
            sides[side] = {
                link: self._entry(side, link).as_dict() for link in ("thigh", "shin")
            }
        return {
            "method": "perpendicular_to_hinge_axis",
            "sides": sides,
            "note": (
                "origin_distance_m is the 3D body-origin distance and is NOT a link"
                " length; the sideways part is axial_offset_m"
            ),
        }


def _leg_links(chain: Mapping[str, ChainLink]) -> tuple[LinkProjection, ...]:
    entries: list[LinkProjection] = []
    for side, thigh_body, shin_body in SIDE_CHAINS:
        for link_name, body in (("thigh", thigh_body), ("shin", shin_body)):
            link = chain.get(body)
            if link is None or link.joint is None:
                raise ValueError(f"{body}: not found in the model, cannot project its link")
            parts = project_offset(link.offset_base_m, link.axis_base)
            local_parts = project_offset(link.offset_parent_m, link.axis_local)
            entries.append(
                LinkProjection(
                    side=side,
                    link=link_name,
                    joint=link.joint,
                    body=body,
                    offset_3d_m=link.offset_parent_m,
                    offset_base_m=link.offset_base_m,
                    axis_base=link.axis_base,
                    planar_length_m=parts["planar_length_m"],
                    axial_offset_m=parts["axial_offset_m"],
                    # same offset expressed in the URDF's own parent frame: the
                    # left leg's local +Z points the other way, so the sign of a
                    # sideways offset depends on the frame it is quoted in
                    axial_offset_local_m=local_parts["axial_offset_m"],
                )
            )
    return tuple(entries)


def resolve_nodes(
    mechanism: Mechanism, chain: Mapping[str, ChainLink] | None = None
) -> ResolvedNodes:
    """CAD node table + the projection checks that it is the right table.

    The nodes are returned **unchanged**: with the projection done correctly the
    model and the STEP measurement agree, so there is nothing to reconcile and no
    second set of lengths to keep in sync.
    """
    links = mjcf_chain() if chain is None else chain
    projections = _leg_links(links)

    cad = {name: node.yz_mm for name, node in mechanism.nodes.items()}
    cad_lengths = {
        "thigh": math.dist(cad["HIP"], cad["KNEE"]),
        "shin": math.dist(cad["KNEE"], cad["WHEEL"]),
    }
    for entry in projections:
        error = abs(entry.planar_length_m * 1000.0 - cad_lengths[entry.link])
        if error > PLANAR_TOLERANCE_MM:
            raise ValueError(
                f"{entry.side} {entry.link}: projected planar length "
                f"{entry.planar_length_m * 1000.0:.9f} mm disagrees with the CAD node"
                f" span {cad_lengths[entry.link]:.9f} mm by {error:.3e} mm; the model"
                " and the planar table are no longer describing the same mechanism"
            )

    resolved = ResolvedNodes(
        nodes_mm=cad,
        projections=projections,
        cad_planar_mm=cad_lengths,
        end_link_mm=math.dist(cad["LINK_PIN_C"], cad["END_PIN_A"]),
    )
    authority(mechanism, resolved)
    return resolved


def authority(
    mechanism: Mechanism, resolved: ResolvedNodes | None = None
) -> tuple[float, float]:
    """The declared planar lengths, verified against the projection.

    ``meta.link_length_authority.lengths`` states what the planar model believes.
    It is not taken on trust: a value that no longer matches the projection means
    somebody edited one of the two and not the other, which is exactly the failure
    this module exists to prevent.
    """
    block = (mechanism.meta.get("link_length_authority") or {}) if mechanism.meta else {}
    lengths = block.get("lengths") or {}
    try:
        declared = (float(lengths["thigh_m"]) * 1000.0, float(lengths["shin_m"]) * 1000.0)
    except (KeyError, TypeError) as error:  # pragma: no cover - config bug
        raise ValueError(
            "meta.link_length_authority.lengths must carry thigh_m and shin_m"
        ) from error

    table = resolved if resolved is not None else resolve_nodes(mechanism)
    projected = (table.thigh_mm, table.shin_mm)
    for name, want, got in zip(("thigh", "shin"), declared, projected):
        if abs(want - got) > PLANAR_TOLERANCE_MM:
            raise ValueError(
                f"{name}: declared authority {want:.9f} mm differs from URDF projection"
                f" {got:.9f} mm; update meta.link_length_authority instead of letting"
                " two length sets coexist"
            )
    return declared


def describe(resolved: ResolvedNodes) -> dict[str, object]:
    """JSON-able summary for the reports."""
    per_side: dict[str, dict[str, object]] = {}
    for side, _, _ in SIDE_CHAINS:
        per_side[side] = {
            "thigh_planar_mm": round(resolved.planar_mm(side, "thigh"), 9),
            "shin_planar_mm": round(resolved.planar_mm(side, "shin"), 9),
            "lateral": {key: round(value, 9) for key, value in resolved.lateral_mm(side).items()},
        }
    projection = resolved.urdf_projection
    sides = projection["sides"]
    return {
        "thigh_mm": round(resolved.thigh_mm, 9),
        "shin_mm": round(resolved.shin_mm, 9),
        "cad_planar_mm": {
            key: round(value, 9) for key, value in resolved.cad_planar_mm.items()
        },
        "per_side": per_side,
        "urdf_projection": {
            "method": projection["method"],
            "sides": {
                side: {
                    link: {
                        key: (round(value, 9) if isinstance(value, float) else value)
                        for key, value in row.items()
                    }
                    for link, row in rows.items()
                }
                for side, rows in sides.items()
            },
        },
        "offset_3d_mm": {
            f"{entry.side}:{entry.link}": round(entry.origin_distance_m * 1000.0, 9)
            for entry in resolved.projections
        },
        "end_link_mm": round(resolved.end_link_mm, 9),
        "end_link_cad_mm": round(resolved.end_link_cad_mm, 9),
        "end_link_change_mm": 0.0,
        "moved_mm": {},
        "worst_move_mm": 0.0,
        "worst_planar_error_mm": round(resolved.worst_planar_error_mm, 12),
        "note": (
            "planar = perpendicular to the hinge axis; the 3D distances"
            " (210.315 / 253.947 mm) include a sideways mounting offset and are"
            " not link lengths"
        ),
    }


def serial_default_pose() -> dict[str, object]:
    """Where the *old* serial model's default joint angles put the wheel.

    The closed-chain reference pose has the wheel directly below the hip.  The
    default angles in ``robot.yaml`` do not - they put it 22.865 mm ahead - and
    the two must not be described as the same pose.  Computed from the model file
    with the standard library, so it needs no MuJoCo.
    """
    from ..config import load_configs
    from .conventions import mjcf_body_positions

    config = load_configs()
    positions = mjcf_body_positions(dict(config.default_angles))
    result: dict[str, object] = {}
    for key, hip_body, knee_body, wheel_body in (
        ("right", "r0_link", "r1_link", "right_wheel"),
        ("left", "l0_link", "l1_link", "left_wheel"),
    ):
        hip = positions[hip_body]
        knee = positions[knee_body]
        wheel = positions[wheel_body]
        result[key] = {
            "thigh_deg": round(
                math.degrees(math.atan2(knee[2] - hip[2], knee[0] - hip[0])), 9
            ),
            "shin_deg": round(
                math.degrees(math.atan2(wheel[2] - knee[2], wheel[0] - knee[0])), 9
            ),
            "wheel_forward_of_hip_mm": round((wheel[0] - hip[0]) * 1000.0, 6),
            "wheel_below_hip_mm": round((hip[2] - wheel[2]) * 1000.0, 6),
            "chord_mm": round(math.hypot(wheel[0] - hip[0], wheel[2] - hip[2]) * 1000.0, 6),
            "lateral_mm": round((wheel[1] - hip[1]) * 1000.0, 6),
        }
    return result


def serial_joint_angles(thigh_rad: float, shin_rad: float) -> dict[str, float]:
    """Absolute planar link angles -> the *serial* model's hinge values.

    Verified against the model at the default pose to 3.6e-15 rad::

        q_rf0 = -thigh
        q_rf1 = thigh - shin - pi/2

    These are the *serial* joints.  The closed chain's ``rf1`` drives the crank
    instead, so these numbers must never be copied into its default.
    """
    return {
        "rf0_Joint": -thigh_rad,
        "rf1_Joint": thigh_rad - shin_rad - math.pi / 2.0,
        "lf0_Joint": thigh_rad,
        "lf1_Joint": -(thigh_rad - shin_rad - math.pi / 2.0),
    }

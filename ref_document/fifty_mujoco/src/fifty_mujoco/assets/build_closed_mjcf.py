"""Candidate closed-chain training model, generated from the confirmed geometry.

The first version of this module emitted a *skeleton*: its job was to prove that
the two loops close.  This one is meant to be **trainable**, so it inherits what
the serial model already gets right instead of inventing it:

* **mass, centre of mass and full inertia** are copied from the serial model's
  ``r0_link`` / ``r1_link`` / ``wheel`` bodies and rotated into the new body
  frames.  The four loop bodies get a small share taken **out of** the thigh and
  shin, so the leg total is unchanged and nothing is counted twice;
* **collision geometry** is copied the same way, so the wheel is the real 0.06 m
  cylinder with the real width rather than a placeholder sphere;
* the **gas spring** uses the confirmed anchors plus the measured 11 mm lateral
  gap, so its tendon length is the spatial ``sqrt(planar² + 11²)`` the reports
  quote;
* the reference pose is written as a **keyframe**, so a reset starts at a closed,
  non-singular configuration instead of converging to one.

Why the rotations: a serial body carries a ``quat`` (the URDF zero pose is not
the CAD assembly pose), while these bodies are unrotated at ``qpos = 0``.  Body
origins are the same physical pivots, so only the *orientation* changes:

    pos_new = R_old · pos_old          quat_new = R_old ⊗ quat_old

Still provisional, and written down rather than hidden: hinge ranges are generous
boxes (the real feasible set is unknown), the spring force is a constant 450 N
with no stiffness, and the mass split inside a link is a modelling choice.

Nothing here runs MuJoCo.  ``verify()`` compiles the result and needs the
environment; the geometry is checked without it by ``tests/test_closed_chain.py``.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from ..mechanism.closed_pose import (
    AXIS_SIGN,
    BODIES,
    DRIVEN,
    JOINT_NAMES,
    SIDES,
    ClosedPose,
    reference_pose,
)
from ..mechanism.conventions import _quat_mul, _quat_rotate, mjcf_chain
from ..mechanism.loader import Mechanism, load_mechanism
from ..mechanism.resolve import describe as describe_resolution, resolve_nodes
from ..paths import assets_dir

Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

#: Serial-model bodies each new body inherits from: ``(right, left)``.
INHERIT: Mapping[str, tuple[str, str]] = {
    "thigh": ("r0_link", "l0_link"),
    "shin": ("r1_link", "l1_link"),
    "wheel": ("right_wheel", "left_wheel"),
}

#: Loop bodies that have no serial counterpart.
LOOP_BODIES = ("crank", "push_rod", "ternary", "end_link")

#: Mass pulled out of the thigh and shin for the loop bodies, kg each.
LOOP_BODY_MASS_KG = 0.030

#: Thin-rod radius used for the loop bodies' inertia, metres.
LOOP_BODY_RADIUS_M = 0.008

#: Nominal standing height, metres (== ``joint_conventions.yaml`` target).
STANDING_HEIGHT_M = 0.4647513721610853

#: Constant spring force for the first training run, newtons.
SPRING_FORCE_N = 450.0

#: Measured lateral gap between the two clevis pins, millimetres.
SPRING_LATERAL_MM = 11.0

#: Generous provisional hinge range for the driven joints, radians.
DRIVEN_RANGE_RAD = 3.2

#: Motor limits copied from the serial model (provisional there as well).
LEG_TORQUE_NM = 30.0
WHEEL_TORQUE_NM = 5.0

MODEL_NAME = "fifty_closed_train"

#: ``(body, parent)`` spanning tree; the two loops are equalities.
TREE: tuple[tuple[str, str], ...] = (
    ("thigh", "trunk"),
    ("shin", "thigh"),
    ("wheel", "shin"),
    ("crank", "trunk"),
    ("push_rod", "crank"),
    ("ternary", "push_rod"),
    ("end_link", "ternary"),
)

#: ``(site1, site2)`` pairs that must coincide, i.e. the closed loops.
LOOPS: tuple[tuple[tuple[str, str], tuple[str, str]], ...] = (
    (("ternary", "LINK_PIN_A"), ("thigh", "LINK_PIN_A")),
    (("end_link", "END_PIN_A"), ("shin", "END_PIN_A")),
)

#: Loop order inside ``qpos`` for one leg, matching how the bodies are emitted.
QPOS_ORDER = ("thigh", "shin", "wheel", "crank", "push_rod", "ternary", "end_link")


# --------------------------------------------------------------------- helpers
def _fmt(value: float) -> str:
    text = repr(float(value))
    return "0" if text == "-0.0" else text


def _vec(values: Any) -> str:
    return " ".join(_fmt(v) for v in values)


def _numbers(text: str | None, count: int, default: Vec3) -> Vec3:
    if not text:
        return default
    parts = [float(part) for part in text.replace(",", " ").split()]
    if len(parts) < count:
        return default
    return (parts[0], parts[1], parts[2])


def _quat_of(text: str | None) -> Quat:
    if not text:
        return (1.0, 0.0, 0.0, 0.0)
    parts = [float(part) for part in text.replace(",", " ").split()]
    if len(parts) != 4:
        return (1.0, 0.0, 0.0, 0.0)
    return (parts[0], parts[1], parts[2], parts[3])


def rotate_inertia(inertia: tuple[float, ...], quat: Quat) -> tuple[float, ...]:
    """Rotate a MuJoCo ``fullinertia`` ``(ixx iyy izz ixy ixz iyz)`` by ``quat``.

    Treats the six numbers as a symmetric tensor in the body frame and returns the
    components in the rotated frame, ``R·I·Rᵀ``.  Used to move the serial model's
    inertia into the new (unrotated) body frame without changing the masses; the
    trace is invariant, which is what ``tests/test_closed_chain.py`` checks.
    """
    ixx, iyy, izz, ixy, ixz, iyz = inertia
    tensor = ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz))
    # columns of R are the rotated basis vectors
    basis = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
    rotation = [_quat_rotate(quat, axis) for axis in basis]
    # rotated = R · tensor · Rᵀ
    first = [
        [sum(rotation[row][k] * tensor[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]
    rotated = [
        [sum(first[row][k] * rotation[column][k] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]
    return (
        rotated[0][0],
        rotated[1][1],
        rotated[2][2],
        rotated[0][1],
        rotated[0][2],
        rotated[1][2],
    )


def inertia_trace(inertia: tuple[float, ...]) -> float:
    return float(inertia[0] + inertia[1] + inertia[2])


@dataclass(frozen=True)
class SerialAsset:
    """What the serial model already gets right, per body."""

    body: str
    quat_base: Quat
    mass_kg: float
    inertial_pos_parent: Vec3
    fullinertia: tuple[float, ...]
    geoms: tuple[Mapping[str, str], ...]

    @property
    def com_offset_base_m(self) -> Vec3:
        """COM offset from the body origin, in the base frame."""
        return _quat_rotate(self.quat_base, self.inertial_pos_parent)


def load_serial_assets(path: Path | None = None) -> dict[str, SerialAsset]:
    """Read masses, inertias and collision geoms out of ``fifty_train.xml``."""
    target = path if path is not None else assets_dir() / "mjcf" / "fifty_train.xml"
    if not target.is_file():
        raise FileNotFoundError(f"serial model not built: {target}")
    chain = mjcf_chain(target)
    root = ET.parse(target).getroot()
    assets: dict[str, SerialAsset] = {}
    for body in root.iter("body"):
        name = body.get("name")
        if not name or name not in chain:
            continue
        inertial = body.find("inertial")
        if inertial is None:
            continue
        raw = inertial.get("fullinertia")
        if raw is not None:
            parts = [float(part) for part in raw.replace(",", " ").split()]
            values = tuple(parts[:6]) if len(parts) >= 6 else (0.0,) * 6
        else:
            diagonal = _numbers(inertial.get("diaginertia"), 3, (0.0, 0.0, 0.0))
            values = (diagonal[0], diagonal[1], diagonal[2], 0.0, 0.0, 0.0)
        assets[name] = SerialAsset(
            body=name,
            quat_base=chain[name].quat_base,
            mass_kg=float(inertial.get("mass", "0")),
            inertial_pos_parent=_numbers(inertial.get("pos"), 3, (0.0, 0.0, 0.0)),
            fullinertia=values,
            geoms=tuple(
                dict(geom.attrib)
                for geom in body.findall("geom")
                if geom.get("class") == "collision"
            ),
        )
    return assets


# -------------------------------------------------------------------- geometry
@dataclass
class LegGeometry:
    """Local-frame geometry of one leg, all in metres."""

    side: str
    hip_y_m: float
    axis_y: float
    lateral_m: dict[str, float] = field(default_factory=dict)
    local: dict[str, dict[str, Vec2]] = field(default_factory=dict)
    pivots: dict[str, tuple[str, Vec2]] = field(default_factory=dict)
    spans: dict[str, float] = field(default_factory=dict)


def leg_geometry(
    side: str,
    nodes_mm: Mapping[str, Vec2],
    hip_y_m: float,
    lateral_mm: Mapping[str, float],
) -> LegGeometry:
    """Body frames equal the trunk frame at ``qpos = 0``.

    So the local coordinates are plain node differences, and the URDF's sideways
    offsets appear as body ``pos`` components.  A hinge turns about Y, which
    preserves each body's own Y, so the loops stay closed for every pose.
    """
    geometry = LegGeometry(side=side, hip_y_m=hip_y_m, axis_y=AXIS_SIGN[side])
    geometry.lateral_m = {
        "thigh": 0.0,
        "crank": 0.0,
        "push_rod": 0.0,
        "ternary": 0.0,
        "end_link": 0.0,
        "shin": float(lateral_mm.get("knee_from_hip_mm", 0.0)) / 1000.0,
        "wheel": float(lateral_mm.get("wheel_from_hip_mm", 0.0)) / 1000.0,
    }
    proximal_of: dict[str, str] = {}
    for body, proximal, carried in BODIES:
        proximal_of[body] = proximal
        anchor = nodes_mm[proximal]
        geometry.local[body] = {
            node: (
                (nodes_mm[node][0] - anchor[0]) / 1000.0,
                (nodes_mm[node][1] - anchor[1]) / 1000.0,
            )
            for node in carried
        }
        geometry.spans[body] = (
            math.dist(anchor, nodes_mm[carried[0]]) / 1000.0 if carried else 0.0
        )
    for name, parent in TREE:
        anchor = nodes_mm[proximal_of[name]]
        parent_anchor = nodes_mm[proximal_of[parent]] if parent in proximal_of else anchor
        geometry.pivots[name] = (
            parent,
            (
                (anchor[0] - parent_anchor[0]) / 1000.0,
                (anchor[1] - parent_anchor[1]) / 1000.0,
            ),
        )
    return geometry


def spring_sites(
    mechanism: Mechanism, geometry: LegGeometry, nodes_mm: Mapping[str, Vec2]
) -> dict[str, Vec3] | None:
    """Local anchor positions for the two clevis pins.

    Only the **difference** between the pins is measured (11 mm lateral), not their
    absolute lateral placement, so the upper pin is put in the thigh's plane and the
    lower follows from the gap.  The resulting 3D length is exactly the
    ``sqrt(planar² + 11²)`` the spring reports use.
    """
    block = (mechanism.topology.get("gas_spring") or {}) if mechanism.topology else {}
    endpoints = block.get("endpoints") or {}
    upper = endpoints.get("upper") or {}
    lower = endpoints.get("lower") or {}
    if not upper.get("point_yz_mm") or not lower.get("point_yz_mm"):
        return None
    hip = nodes_mm["HIP"]
    knee = nodes_mm["KNEE"]
    upper_local = (
        (float(upper["point_yz_mm"][0]) - hip[0]) / 1000.0,
        0.0,
        (float(upper["point_yz_mm"][1]) - hip[1]) / 1000.0,
    )
    lower_lateral = SPRING_LATERAL_MM / 1000.0 - geometry.lateral_m["shin"]
    lower_local = (
        (float(lower["point_yz_mm"][0]) - knee[0]) / 1000.0,
        lower_lateral,
        (float(lower["point_yz_mm"][1]) - knee[1]) / 1000.0,
    )
    return {"upper": upper_local, "lower": lower_local}


def mass_plan(assets: Mapping[str, SerialAsset], side_key: str) -> dict[str, float]:
    """Loop bodies are paid for by the thigh and shin in proportion to their mass."""
    thigh = assets["r0_link" if side_key == "r" else "l0_link"].mass_kg
    shin = assets["r1_link" if side_key == "r" else "l1_link"].mass_kg
    wheel = assets["right_wheel" if side_key == "r" else "left_wheel"].mass_kg
    loop = {name: LOOP_BODY_MASS_KG for name in LOOP_BODIES}
    spare = sum(loop.values())
    if spare >= thigh + shin:
        raise ValueError("the loop-body share exceeds the thigh+shin mass")
    share = thigh / (thigh + shin)
    masses = {
        "thigh": thigh - spare * share,
        "shin": shin - spare * (1.0 - share),
        "wheel": wheel,
        **loop,
    }
    if abs(sum(masses.values()) - (thigh + shin + wheel)) > 1e-12:
        raise AssertionError("mass is not conserved")
    return masses


def loop_inertial(span_m: float, mass_kg: float) -> dict[str, str]:
    if span_m <= 1e-9:
        moment = 0.4 * mass_kg * LOOP_BODY_RADIUS_M**2
        return {
            "pos": "0 0 0",
            "mass": _fmt(mass_kg),
            "diaginertia": _vec((moment, moment, moment)),
        }
    axial = 0.5 * mass_kg * LOOP_BODY_RADIUS_M**2
    transverse = mass_kg * (3.0 * LOOP_BODY_RADIUS_M**2 + span_m**2) / 12.0
    return {
        "pos": _vec((span_m / 2.0, 0.0, 0.0)),
        "mass": _fmt(mass_kg),
        "diaginertia": _vec((axial, transverse, transverse)),
    }


# ----------------------------------------------------------------------- build
def build_model(
    mechanism: Mechanism | None = None,
    serial_path: Path | None = None,
    standing_height_m: float = STANDING_HEIGHT_M,
) -> tuple[ET.Element, dict[str, Any]]:
    """Return the MJCF root element plus the numbers used to author it."""
    mech = mechanism if mechanism is not None else load_mechanism()
    resolved = resolve_nodes(mech)
    nodes_mm = dict(resolved.nodes_mm)
    assets = load_serial_assets(serial_path)
    pose = reference_pose(mech)

    root = ET.Element("mujoco", {"model": MODEL_NAME})
    ET.SubElement(
        root,
        "compiler",
        {"angle": "radian", "autolimits": "true", "balanceinertia": "false"},
    )
    ET.SubElement(
        root,
        "option",
        {"timestep": "0.005", "integrator": "implicitfast", "cone": "elliptic"},
    )
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", {"damping": "0.03", "frictionloss": "0.015"})
    collision = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(
        collision,
        "geom",
        {
            "group": "3",
            "contype": "2",
            "conaffinity": "1",
            "friction": "1.0 0.005 0.0001",
            "solref": "0.02 1.0",
            "solimp": "0.9 0.95 0.001",
        },
    )
    for name in ("hip", "knee", "wheel"):
        entry = ET.SubElement(default, "default", {"class": name})
        if name == "wheel":
            ET.SubElement(
                entry,
                "joint",
                {"damping": "0.002", "frictionloss": "0.001", "armature": "0.01"},
            )
        else:
            ET.SubElement(entry, "joint", {"damping": "0.03", "frictionloss": "0.015"})

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(
        world,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "size": "10 10 0.1",
            "pos": "0 0 0",
            "contype": "1",
            "conaffinity": "1",
            "friction": "1.0 0.005 0.0001",
            "group": "1",
        },
    )
    trunk = ET.SubElement(
        world, "body", {"name": "base_link", "pos": _vec((0.0, 0.0, standing_height_m))}
    )
    ET.SubElement(trunk, "freejoint", {"name": "root"})
    base = assets["base_link"]
    ET.SubElement(
        trunk,
        "inertial",
        {
            "pos": _vec(base.inertial_pos_parent),
            "mass": _fmt(base.mass_kg),
            "fullinertia": _vec(base.fullinertia),
        },
    )
    for geom in base.geoms:
        ET.SubElement(trunk, "geom", dict(geom))

    authored: dict[str, Any] = {"legs": {}, "masses": {}, "spring": {}, "note": ""}
    for side, key in SIDES:
        geometry = leg_geometry(
            side, nodes_mm, _hip_lateral_m(side), resolved.lateral_mm(side)
        )
        masses = mass_plan(assets, key)
        authored["masses"][side] = masses
        authored["legs"][side] = {
            "hip_lateral_m": geometry.hip_y_m,
            "knee_from_hip_mm": geometry.lateral_m["shin"] * 1000.0,
            "wheel_from_hip_mm": geometry.lateral_m["wheel"] * 1000.0,
        }
        anchored = spring_sites(mech, geometry, nodes_mm)
        bodies: dict[str, ET.Element] = {}
        for name, parent in TREE:
            _, offset = geometry.pivots[name]
            lateral = geometry.lateral_m[name]
            pos = (
                (0.0, geometry.hip_y_m + lateral, 0.0)
                if parent == "trunk"
                else (offset[0], lateral - geometry.lateral_m[parent], offset[1])
            )
            element = ET.SubElement(
                bodies.get(parent, trunk), "body", {"name": f"{key}_{name}", "pos": _vec(pos)}
            )
            bodies[name] = element

            inherited = _inherited(assets, name, key)
            if inherited is None:
                ET.SubElement(element, "inertial", loop_inertial(geometry.spans[name], masses[name]))
            else:
                ET.SubElement(
                    element,
                    "inertial",
                    {
                        "pos": _vec(inherited.com_offset_base_m),
                        "mass": _fmt(masses[name]),
                        "fullinertia": _vec(
                            rotate_inertia(inherited.fullinertia, inherited.quat_base)
                        ),
                    },
                )
                for geom in inherited.geoms:
                    _copy_geom(element, geom, inherited)

            attributes = {
                "name": JOINT_NAMES[name].format(key=key),
                "type": "hinge",
                # the wheels share one direction on both sides (policy_contract
                # says l_wheel +Y); only the leg hinges mirror
                "axis": _vec((0.0, 1.0 if name == "wheel" else geometry.axis_y, 0.0)),
            }
            if name == "wheel":
                attributes["class"] = "wheel"
                attributes["limited"] = "false"
            elif name in DRIVEN:
                attributes["class"] = "knee" if name in ("shin", "end_link") else "hip"
                attributes["range"] = _vec((-DRIVEN_RANGE_RAD, DRIVEN_RANGE_RAD))
            else:
                attributes["limited"] = "false"
            ET.SubElement(element, "joint", attributes)

            for node, local in geometry.local[name].items():
                site_lateral = lateral
                if node == "LINK_PIN_A":
                    site_lateral = geometry.lateral_m["thigh"]
                elif name == "end_link" and node == "END_PIN_A":
                    site_lateral = geometry.lateral_m["shin"]
                ET.SubElement(
                    element,
                    "site",
                    {
                        "name": f"{key}_{name}_{node}",
                        "pos": _vec((local[0], site_lateral - lateral, local[1])),
                        "size": "0.004",
                    },
                )

        if anchored is not None:
            ET.SubElement(
                bodies["thigh"],
                "site",
                {"name": f"{key}_gas_a", "pos": _vec(anchored["upper"]), "size": "0.004"},
            )
            ET.SubElement(
                bodies["shin"],
                "site",
                {"name": f"{key}_gas_b", "pos": _vec(anchored["lower"]), "size": "0.004"},
            )
            authored["spring"][side] = {
                "upper_local_m": [round(v, 9) for v in anchored["upper"]],
                "lower_local_m": [round(v, 9) for v in anchored["lower"]],
                "lateral_gap_mm": round(
                    (geometry.lateral_m["shin"] + anchored["lower"][1]) * 1000.0
                    - (geometry.lateral_m["thigh"] + anchored["upper"][1]) * 1000.0,
                    9,
                ),
                "force_n": SPRING_FORCE_N,
                "model": "constant_force_placeholder",
            }

    equality = ET.SubElement(root, "equality")
    for side, key in SIDES:
        for first, second in LOOPS:
            ET.SubElement(
                equality,
                "connect",
                {
                    "name": f"{key}_{first[0]}_{first[1]}",
                    "site1": f"{key}_{first[0]}_{first[1]}",
                    "site2": f"{key}_{second[0]}_{second[1]}",
                    "solref": "0.004 1",
                },
            )

    if authored["spring"]:
        tendon = ET.SubElement(root, "tendon")
        for side, key in SIDES:
            ET.SubElement(
                tendon,
                "spatial",
                {"name": f"gas_{side}", "site1": f"{key}_gas_a", "site2": f"{key}_gas_b", "width": "0.006"},
            )

    actuator = ET.SubElement(root, "actuator")
    for side, key in SIDES:
        for body, limit in (
            ("thigh", LEG_TORQUE_NM),
            ("crank", LEG_TORQUE_NM),
            ("wheel", WHEEL_TORQUE_NM),
        ):
            name = JOINT_NAMES[body].format(key=key)
            ET.SubElement(
                actuator,
                "motor",
                {
                    "name": f"{name}_motor",
                    "joint": name,
                    "ctrllimited": "true",
                    "forcelimited": "true",
                    "ctrlrange": _vec((-limit, limit)),
                    "forcerange": _vec((-limit, limit)),
                },
            )
        if authored["spring"]:
            ET.SubElement(
                actuator,
                "motor",
                {
                    "name": f"gas_{side}_force",
                    "tendon": f"gas_{side}",
                    "gear": "1",
                    "ctrllimited": "true",
                    "ctrlrange": _vec((-SPRING_FORCE_N, SPRING_FORCE_N)),
                    "forcelimited": "true",
                    "forcerange": _vec((-SPRING_FORCE_N, SPRING_FORCE_N)),
                },
            )

    keyframe = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframe, "key", keyframe_attributes(pose, standing_height_m))

    authored["note"] = (
        "masses, inertias and collision geoms are inherited from the serial model and"
        f" rotated into these body frames; {4 * LOOP_BODY_MASS_KG:.3f} kg of loop bodies"
        " is taken out of the thigh and shin so the leg total is unchanged"
    )
    authored["resolution"] = describe_resolution(resolved)
    authored["reference_pose"] = {
        "qpos_rad": {body: round(value, 12) for body, value in pose.qpos.items()},
        "chord_m": round(pose.chord_m, 12),
    }
    authored["joint_ranges"] = {
        "driven": [-DRIVEN_RANGE_RAD, DRIVEN_RANGE_RAD],
        "passive": "unlimited",
        "wheel": "unlimited",
        "note": "provisional boxes; the real closed-chain feasible set is not known",
    }
    return root, authored


def keyframe_attributes(pose: ClosedPose, standing_height_m: float) -> dict[str, str]:
    """``qpos`` of the reference key: free joint then the two legs' seven hinges."""
    values = [0.0, 0.0, standing_height_m, 1.0, 0.0, 0.0, 0.0]
    for _, key in SIDES:
        per_side = pose.joint_values("right" if key == "r" else "left")
        for body in QPOS_ORDER:
            values.append(per_side[JOINT_NAMES[body].format(key=key)])
    return {"name": "reference", "qpos": _vec(values)}


def _hip_lateral_m(side: str) -> float:
    chain = mjcf_chain()
    return chain["r0_link" if side == "right" else "l0_link"].offset_base_m[1]


def _inherited(assets: Mapping[str, SerialAsset], body: str, key: str) -> SerialAsset | None:
    pair = INHERIT.get(body)
    if pair is None:
        return None
    return assets[pair[0] if key == "r" else pair[1]]


def _copy_geom(element: ET.Element, geom: Mapping[str, str], inherited: SerialAsset) -> None:
    """``pos``/``quat`` are written in the old body frame; rotate them into the new one."""
    quat = inherited.quat_base
    attributes = dict(geom)
    attributes["pos"] = _vec(
        _quat_rotate(quat, _numbers(geom.get("pos"), 3, (0.0, 0.0, 0.0)))
    )
    if geom.get("quat"):
        attributes["quat"] = _vec(_quat_mul(quat, _quat_of(geom.get("quat"))))
    ET.SubElement(element, "geom", attributes)


# -------------------------------------------------------------------- io/main
def write(
    path: Path | None = None,
    mechanism: Mechanism | None = None,
    serial_path: Path | None = None,
) -> Path:
    """Emit the candidate model and return where it went."""
    target = path if path is not None else assets_dir() / "mjcf" / f"{MODEL_NAME}.xml"
    root, _ = build_model(mechanism, serial_path)
    ET.indent(root, space="  ")
    target.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(target, encoding="utf-8", xml_declaration=False)
    with target.open("a", encoding="utf-8") as handle:
        handle.write("\n")
    return target


def verify(path: Path | None = None) -> dict[str, Any]:
    """Compile the candidate model and check the loops, springs and keyframe.

    Needs MuJoCo - run it where the environment exists.  Everything the geometry
    can state without compiling is covered by ``tests/test_closed_chain.py``.
    """
    import mujoco
    import numpy as np

    target = path if path is not None else assets_dir() / "mjcf" / f"{MODEL_NAME}.xml"
    model = mujoco.MjModel.from_xml_path(str(target))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_kinematics(model, data)

    def site(name: str) -> Any:
        return data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)].copy()

    residuals: dict[str, float] = {}
    for side, key in SIDES:
        for first, second in LOOPS:
            residuals[f"{side}:{first[0]}.{first[1]}"] = float(
                np.linalg.norm(
                    site(f"{key}_{first[0]}_{first[1]}") - site(f"{key}_{second[0]}_{second[1]}")
                )
            )
    springs: dict[str, float] = {}
    for side in ("right", "left"):
        identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"gas_{side}")
        if identifier >= 0:
            springs[side] = float(data.ten_length[identifier])
    return {
        "model": str(target),
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "neq": int(model.neq),
        "ntendon": int(model.ntendon),
        "nbody": int(model.nbody),
        "total_mass_kg": float(model.body_mass.sum()),
        "loop_residual_m": residuals,
        "worst_loop_residual_m": max(residuals.values()) if residuals else 0.0,
        "spring_length_m": springs,
        "qpos": [float(value) for value in data.qpos],
    }


def main() -> int:
    target = write()
    report = verify()
    print(f"wrote {target}")
    print(
        f"compiled : nq {report['nq']}  nv {report['nv']}  nu {report['nu']}"
        f"  neq {report['neq']}  ntendon {report['ntendon']}"
    )
    print(f"mass     : {report['total_mass_kg']:.4f} kg")
    for name, residual in report["loop_residual_m"].items():
        print(f"  loop   {name}  {residual:.3e} m")
    for side, length in report["spring_length_m"].items():
        print(f"  spring {side:<5}  {length * 1000.0:.6f} mm")
    return 0 if report["worst_loop_residual_m"] < 1e-9 else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())

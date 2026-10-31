"""STEP audit: assembly structure *and* cylindrical-axis candidates.

Why this exists
---------------
``progress.md`` §6.3 lists candidate link lengths (G01 210 mm, G02 250 mm, ...)
that were measured once by an ad-hoc script.  Without a reproducible tool those
numbers cannot be re-derived, checked or extended, so the STEP route stalls.
This module is that tool, and it is deliberately **standard library only**: no
numpy, no MuJoCo, no SolidWorks, so it runs on a bare Python even on a machine
where the simulation stack is not installed.

What it extracts
----------------
* assembly structure: ``PRODUCT`` / ``PRODUCT_DEFINITION`` / ``NEXT_ASSEMBLY_USAGE_OCCURRENCE``
  and the instance transform (``ITEM_DEFINED_TRANSFORMATION``),
* geometry attribution: ``SHAPE_DEFINITION_REPRESENTATION`` -> ``SHAPE_REPRESENTATION``
  -> ``MANIFOLD_SOLID_BREP`` -> ``CLOSED_SHELL`` -> ``ADVANCED_FACE`` ->
  ``CYLINDRICAL_SURFACE``, so every cylinder axis is attributed to a part,
* per-part dominant axes (largest bore first) and the distance between two given
  cylinders, which is how a link length is obtained.

What it still cannot do
-----------------------
* No mass/material: the file has no ``MASS_DENSITY`` / ``PROPERTY_DEFINITION``.
* No mates: ``KINEMATIC_PAIR`` is not exported by SwSTEP; motion axes must be
  chosen by a human in SolidWorks.
* No cross-part distance that depends on the assembly transform: the convention
  ``A_axis1 * inv(A_axis2)`` is unverified, so such results are reported as
  *unmeasured* with the reason, not silently computed.
* A distance computed between two cylinders of the **same** part is
  convention-free (both live in that part's own frame) and is the only kind
  promoted to a candidate link length.

Every geometric value keeps ``provisional`` provenance: geometry is not a
measurement of the assembled robot.
"""

from __future__ import annotations

import hashlib
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

_ENTITY = re.compile(r"#(\d+)\s*=\s*([A-Z0-9_]+)\s*\(", re.ASCII)
_COMPLEX = re.compile(r"#(\d+)\s*=\s*\(", re.ASCII)
_SUBTYPE = re.compile(r"([A-Z0-9_]+)\s*\(([^()]*)\)", re.ASCII)
_X2 = re.compile(r"\\X2\\([0-9A-Fa-f]*)\\X0\\")
_X1 = re.compile(r"\\X\\([0-9A-Fa-f]{2})")

#: Entities whose full parameter text is kept.
KEEP = {
    "PRODUCT",
    "PRODUCT_DEFINITION",
    "PRODUCT_DEFINITION_FORMATION",
    "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE",
    "PRODUCT_DEFINITION_SHAPE",
    "NEXT_ASSEMBLY_USAGE_OCCURRENCE",
    "ITEM_DEFINED_TRANSFORMATION",
    "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION",
    "SHAPE_REPRESENTATION_RELATIONSHIP",
    "REPRESENTATION_RELATIONSHIP",
    "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
    "SHAPE_DEFINITION_REPRESENTATION",
    "SHAPE_REPRESENTATION",
    "ADVANCED_BREP_SHAPE_REPRESENTATION",
    "MANIFOLD_SOLID_BREP",
    "CLOSED_SHELL",
    "ADVANCED_FACE",
    "CYLINDRICAL_SURFACE",
    "AXIS2_PLACEMENT_3D",
    "CARTESIAN_POINT",
    "DIRECTION",
    "SI_UNIT",
    "CONVERSION_BASED_UNIT",
    "LENGTH_UNIT",
}

#: Entities that are only counted (never needed for structure or axes).
COUNTED = {
    "BREP_WITH_VOIDS",
    "SHELL_BASED_SURFACE_MODEL",
    "ORIENTED_EDGE",
    "EDGE_CURVE",
    "VERTEX_POINT",
    "PLANE",
    "TOROIDAL_SURFACE",
    "SPHERICAL_SURFACE",
    "CONICAL_SURFACE",
    "B_SPLINE_SURFACE_WITH_KNOTS",
}

#: Candidate link-length pairs from ``progress.md`` §6.3 (entity ids).
#: Reproducing these numbers is the acceptance test for this parser.
DEFAULT_AXIS_PAIRS: tuple[tuple[str, int, int], ...] = (
    ("G01 大腿 髋轴-膝轴", 1002386, 677239),
    ("G02 小腿 膝轴-轮轴", 1183545, 1604776),
    ("G03 前推杆 两轴", 1671119, 823026),
    ("G04 末端连杆 两轴", 1907542, 1556259),
    ("G05 三孔件 A-B", 1567379, 1029446),
    ("G06 三孔件 A-C", 1567379, 2672357),
    ("G07 三孔件 B-C", 1029446, 2672357),
)

#: Spring pin axes (§6.3 G08).  Cross-part distance needs verified transforms.
DEFAULT_CROSS_PART_PAIRS: tuple[tuple[str, int, int], ...] = (
    ("G08 气弹簧 下销-上销", 2141130, 1746206),
)

#: Part names worth reporting axes for (substring match).
INTERESTING_PARTS: tuple[str, ...] = (
    "大腿",
    "小腿",
    "膝关节",
    "髋关节",
    "氮气弹簧",
    "轮腿",
)

#: Drive motors (progress.md §23.2).  The pattern only *finds* candidates; the
#: assembly it sits in decides whether it belongs to the wheel-leg drive train,
#: because the same product name also appears in the gimbal and the shooter.
DRIVE_MOTOR_PATTERNS: tuple[str, ...] = ("DM-8009", "M3508")

#: Only a motor inside a module with this name is part of the leg/wheel drive.
DRIVE_MODULE_ASSEMBLY = "髋关节"

#: Parts that make up the transmission next to the motors.
DRIVE_TRANSMISSION_PATTERNS: tuple[str, ...] = (
    "主动链轮",
    "从动链轮",
    "GEARBOX",
    "GearAssembly",
    "T3508_Gear",
)

#: Gas spring (progress.md §23.3): the two halves and the two seat ends.
GAS_SPRING_PART_PATTERNS: tuple[str, ...] = ("1BKB",)
#: The spring's end mounts: their pin bores are the two anchor points (G08).
GAS_SPRING_CLEVIS_PATTERNS: tuple[str, ...] = ("弹簧套",)
#: Assemblies and drivetrain parts that carry a motor name without being one.
DRIVE_EXCLUDE_PATTERNS: tuple[str, ...] = (
    "Gear_MKIV_III",
    "GearAssembly",
    "分线板",
)
#: The two pins whose carrier decides the fixed-link variant (progress.md §21.4).
#: Coordinates are copied from ``configs/mechanism.yaml: planar_nodes`` (both are
#: ``cad_extracted``); the audit is standard-library only and cannot read YAML.
VARIANT_PINS: dict[str, tuple[float, float]] = {
    "LINK_PIN_A": (-15.794862355, 76.513320082),
    "END_PIN_A": (-40.257800194, 185.684300943),
}

#: Which rigid body a part belongs to, from its name.  The drivetrain check comes
#: first on purpose: ``小腿传动中部连杆`` contains 小腿 but is not the shin.
BODY_NAME_MARKERS: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("drivetrain", ("传动", "连杆", "轴承座", "轴承", "限位")),
    ("thigh", ("大腿",)),
    ("shin", ("小腿",)),
    ("wheel", ("轮腿",)),
)


def classify_body(part_name: str) -> str:
    for body, markers in BODY_NAME_MARKERS:
        if any(marker in part_name for marker in markers):
            return body
    return "other"


def _pin_carriers(
    by_rep: dict[int, list[dict[str, Any]]],
    rep_names: dict[int, list[str]],
    tolerance_mm: float = 0.5,
) -> dict[str, Any]:
    """Every part with a bore on each variant-deciding pin (progress.md §21.4).

    The shared-node analysis only looks at parts whose name matches a leg
    keyword, so a bearing block, a bearing or a mirror link next to the pin was
    never considered - and those are exactly the parts that fix the pin.  This
    scans *all* attributed cylinders whose axis passes within ``tolerance_mm`` of
    the pin, then classifies each part by the rigid body it belongs to.

    Still ``candidate``: a coincident bore proves the parts share an axis, not
    that a mate pins them together.
    """
    report: dict[str, Any] = {"pins": {}, "status": "candidate", "tolerance_mm": tolerance_mm}
    for name, (target_y, target_z) in VARIANT_PINS.items():
        found: dict[str, dict[str, Any]] = {}
        for rep_id, cylinders in by_rep.items():
            names = rep_names.get(rep_id) or []
            if not names:
                continue
            for entry in cylinders:
                axis = entry["axis"]
                if abs(abs(axis[0]) - 1.0) > 1e-6:
                    continue  # only the X-parallel bores belong to this plane
                distance = math.hypot(
                    entry["origin_mm"][1] - target_y, entry["origin_mm"][2] - target_z
                )
                if distance > tolerance_mm:
                    continue
                record = found.setdefault(
                    names[0],
                    {
                        "part": names[0],
                        "body": classify_body(names[0]),
                        "distance_mm": round(distance, 6),
                        "max_radius_mm": 0.0,
                        "coaxial_cylinders": 0,
                        "x_planes_mm": [],
                    },
                )
                record["max_radius_mm"] = round(
                    max(record["max_radius_mm"], entry["radius_mm"]), 6
                )
                record["coaxial_cylinders"] += 1
                plane = round(entry["origin_mm"][0], 3)
                if plane not in record["x_planes_mm"]:
                    record["x_planes_mm"].append(plane)
        parts = sorted(
            found.values(), key=lambda item: (-item["max_radius_mm"], item["part"])
        )
        bodies: dict[str, list[str]] = {}
        for part in parts:
            bodies.setdefault(part["body"], []).append(part["part"])
        report["pins"][name] = {
            "point_mm": [target_y, target_z],
            "parts": parts,
            "by_body": bodies,
            "status": "shared" if len(bodies) > 1 else ("single_part" if parts else "not_found"),
        }

    link_a = report["pins"]["LINK_PIN_A"]["by_body"]
    end_a = report["pins"]["END_PIN_A"]["by_body"]
    if "thigh" in link_a and "shin" not in link_a and "shin" in end_a and "thigh" not in end_a:
        report["variant_supported"] = "link_a_on_thigh"
    elif "shin" in link_a and "thigh" not in link_a and "thigh" in end_a and "shin" not in end_a:
        report["variant_supported"] = "link_a_on_shin"
    else:
        report["variant_supported"] = None
    report["note"] = (
        "the plate that carries the pin decides the variant; the bearing block and "
        "the bearing around the same bore are the corroborating parts, so confirm "
        "the mate in SolidWorks before promoting selected_variant"
    )
    return report

#: ``STD_DualSpurGear_M0.6xT14_M0.6xT46`` carries module and tooth count in the
#: part name.  That is the only place the ratio can come from: SwSTEP exports no
#: ``PROPERTY_DEFINITION`` and no gear mate.
GEAR_TOOTH_PATTERN = re.compile(r"M(?P<module>\d+(?:\.\d+)?)xT(?P<teeth>\d+)")
INTERNAL_GEAR_MARKER = "InternalSpurGear"
SUN_GEAR_MARKERS: tuple[str, ...] = ("3508_Gear",)

#: Sprocket names carry no tooth count, so it has to be counted from the geometry:
#: the tooth seats are coaxial cylinders repeated at exactly 360/N around the
#: boss.  ``链条``/``链轮`` in the name is the only hint that a part is one.
SPROCKET_MARKERS: tuple[str, ...] = ("链轮",)
#: A tooth-seat ring needs at least this many seats before it is believed, and
#: the angular spacing must be uniform to within this fraction of the mean.
SPROCKET_MIN_TEETH = 8
SPROCKET_UNIFORMITY_TOLERANCE = 0.02
#: Standard roller-chain pitches, mm.  A counted tooth count is only reported as
#: confirmed when the implied pitch lands on one of these.
CHAIN_PITCHES_MM: tuple[float, ...] = (6.35, 9.525, 12.7, 15.875, 19.05, 25.4)
CHAIN_PITCH_TOLERANCE_MM = 0.15

#: 2026-10-31: the operator counted the sun gear in SolidWorks.  Recorded as a
#: constant so the centre-distance derivation is checked *against a human
#: reading* instead of only against itself.
CAD_CONFIRMED_SUN_TEETH = 17


def _assembly_path(instances: Sequence[dict[str, Any]], name: str, limit: int = 10) -> dict[str, Any]:
    """Walk the NAUO parents of one product instance up to the root.

    Names repeat in this file (two identical gearbox sub-assemblies), so the
    walk follows the first parent and reports when a node has several - a
    silently invented single path is worse than a flagged ambiguity.
    """
    parents: dict[str, list[str]] = {}
    for entry in instances:
        bucket = parents.setdefault(entry["child"], [])
        if entry["parent"] not in bucket:
            bucket.append(entry["parent"])
    path: list[str] = []
    ambiguous = False
    node = name
    for _ in range(limit):
        options = parents.get(node, [])
        if not options:
            break
        if len(options) > 1:
            ambiguous = True
        parent = options[0]
        if parent in path or parent == node:
            break
        path.append(parent)
        node = parent
    return {"path": path, "ambiguous": ambiguous}


def _part_axes(
    by_rep: dict[int, list[dict[str, Any]]],
    rep_names: dict[int, list[str]],
    patterns: Sequence[str],
) -> list[dict[str, Any]]:
    """Axis groups of every part whose name contains one of ``patterns``."""
    found: list[dict[str, Any]] = []
    for rep_id, cylinders in by_rep.items():
        names = rep_names.get(rep_id) or []
        matched = [name for name in names if any(p in name for p in patterns)]
        if not matched:
            continue
        found.append(
            {
                "rep": rep_id,
                "parts": names[:3],
                "matched": matched[0],
                "cylinder_count": len(cylinders),
                "dominant_axes": _dominant_axes(cylinders, limit=3),
            }
        )
    found.sort(key=lambda item: item["matched"])
    return found


def _gear_teeth(names: Sequence[str]) -> list[dict[str, Any]]:
    """``M0.6xT14`` style tokens in a part name -> ``{module, teeth, internal}``."""
    internal = any(INTERNAL_GEAR_MARKER in name for name in names)
    found: list[dict[str, Any]] = []
    for name in names:
        for match in GEAR_TOOTH_PATTERN.finditer(name):
            entry = {
                "module_mm": float(match.group("module")),
                "teeth": int(match.group("teeth")),
                "internal": internal,
            }
            if entry not in found:
                found.append(entry)
    return found


def _predicted_radii(module: float, teeth: int, internal: bool) -> dict[str, float]:
    """Tip and root radius of a standard 20 deg spur gear (module = mm/tooth)."""
    if internal:
        return {
            "tip_mm": module * (teeth / 2.0 - 1.0),
            "root_mm": module * (teeth / 2.0 + 1.25),
        }
    return {
        "tip_mm": module * (teeth / 2.0 + 1.0),
        "root_mm": module * (teeth / 2.0 - 1.25),
    }


def _chain_pitch(radius_mm: float, teeth: int) -> float:
    """``p = 2 R sin(pi/N)``: the roller-chain pitch implied by a tooth count."""
    return 2.0 * radius_mm * math.sin(math.pi / teeth)


def _nearest_standard_pitch(pitch_mm: float) -> tuple[float, float] | None:
    best = min(CHAIN_PITCHES_MM, key=lambda value: abs(value - pitch_mm))
    error = abs(best - pitch_mm)
    return (best, error) if error <= CHAIN_PITCH_TOLERANCE_MM else None


def _parallel(first: Sequence[float], second: Sequence[float], tolerance: float = 1e-6) -> bool:
    """True when two axes are collinear directions, in either sense.

    A cylindrical surface's axis vector has no preferred sign: the exporter writes
    ``+X`` for one bore and ``-X`` for the same bore on the other face. Comparing
    only the same sense silently drops half the holes on a part - which is exactly
    how the crank sprocket's 19-tooth ring went missing while the thigh's worked.
    """
    a = _unit((first[0], first[1], first[2]))
    b = _unit((second[0], second[1], second[2]))
    same = all(abs(x - y) <= tolerance for x, y in zip(a, b))
    opposite = all(abs(x + y) <= tolerance for x, y in zip(a, b))
    return same or opposite


def _merge_angles(angles: Sequence[float], tolerance_deg: float = 0.5) -> list[float]:
    """Collapse angles that differ only by rounding, including across 0/360."""
    ordered = sorted(angle % 360.0 for angle in angles)
    merged: list[float] = []
    for angle in ordered:
        if merged and angle - merged[-1] <= tolerance_deg:
            continue
        merged.append(angle)
    if len(merged) > 1 and 360.0 - merged[-1] + merged[0] <= tolerance_deg:
        merged.pop()
    return merged


def sprocket_teeth(
    cylinders: Sequence[dict[str, Any]], axis_hint: Vec3 | None = None
) -> dict[str, Any]:
    """Count the teeth of one sprocket from its tooth-seat ring.

    A sprocket stores one coaxial cylinder per tooth seat, all at the pitch radius
    and evenly spaced in angle. That gives three independent numbers - the seat
    radius, the count, and the spacing - so the pitch ``p = 2 R sin(pi/N)`` can be
    checked against the standard chain pitches.

    That check is what separates teeth from lightening holes. The 12 bolt holes in
    a sprocket hub are also a uniform ring at 30 deg spacing, and counting them
    gave a 12-tooth "sprocket" with a 11.1 mm pitch - no such chain exists. A ring
    is therefore accepted only when its implied pitch is a standard one.
    """
    if not cylinders:
        return {"status": "no_cylinders", "teeth": None}
    dominant = _dominant_axes(cylinders, limit=1)
    if not dominant:
        return {"status": "no_axis", "teeth": None}
    axis = (
        tuple(axis_hint) if axis_hint is not None else tuple(dominant[0]["axis"])
    )
    center = dominant[0]["origin_mm"]

    rings: dict[float, list[float]] = {}
    for entry in cylinders:
        if not _parallel(tuple(entry["axis"]), axis):
            continue
        # the sprockets of this assembly all turn about X, so the perpendicular
        # plane is (Y, Z); keeping it explicit avoids a silent index choice
        y = entry["origin_mm"][1] - center[1]
        z = entry["origin_mm"][2] - center[2]
        radius = math.hypot(y, z)
        if radius < 1.0:
            continue
        angle = math.degrees(math.atan2(z, y))
        rings.setdefault(round(radius, 2), []).append(angle)

    candidates: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    for radius, angles in rings.items():
        merged = _merge_angles(angles)
        if len(merged) < SPROCKET_MIN_TEETH:
            continue
        gaps = [
            (merged[(index + 1) % len(merged)] - merged[index]) % 360.0
            for index in range(len(merged))
        ]
        mean = sum(gaps) / len(gaps)
        spread = max(abs(gap - mean) for gap in gaps)
        pitch = _chain_pitch(radius, len(merged))
        standard = _nearest_standard_pitch(pitch)
        record = {
            "pitch_radius_mm": round(radius, 3),
            "teeth": len(merged),
            "angular_spacing_deg": round(mean, 4),
            "spacing_spread_deg": round(spread, 4),
            "implied_chain_pitch_mm": round(pitch, 4),
            "standard_pitch_mm": None if standard is None else standard[0],
            "pitch_error_mm": None if standard is None else round(standard[1], 4),
        }
        uniform = mean > 0.0 and spread / mean <= SPROCKET_UNIFORMITY_TOLERANCE
        if uniform and standard is not None:
            record["status"] = "confirmed"
            candidates.append(record)
        else:
            record["status"] = (
                "non_uniform_ring" if not uniform else "no_standard_pitch"
            )
            rejected.append(record)
    if not candidates:
        return {
            "status": "no_tooth_ring",
            "teeth": None,
            "rejected_rings": sorted(
                rejected, key=lambda item: item["pitch_radius_mm"], reverse=True
            )[:4],
        }
    # copy, not mutate: ``best`` is one of the records inside ``candidates``,
    # so attaching the list to it would make the record contain itself and the
    # report would no longer be JSON-serialisable
    best = dict(max(candidates, key=lambda item: item["pitch_radius_mm"]))
    best["candidates"] = sorted(
        candidates, key=lambda item: item["pitch_radius_mm"], reverse=True
    )
    best["rejected_rings"] = sorted(
        rejected, key=lambda item: item["pitch_radius_mm"], reverse=True
    )[:4]
    return best


def _sprocket_train(
    by_rep: dict[int, list[dict[str, Any]]],
    rep_names: dict[int, list[str]],
) -> dict[str, Any]:
    """Tooth counts of the leg chain drives and the ratio they imply."""
    entries: list[dict[str, Any]] = []
    for rep_id, cylinders in by_rep.items():
        names = rep_names.get(rep_id) or []
        matched = [name for name in names if any(m in name for m in SPROCKET_MARKERS)]
        if not matched:
            continue
        counted = sprocket_teeth(cylinders)
        entries.append(
            {
                "rep": rep_id,
                "part": matched[0],
                "role": (
                    "driving"
                    if "主动" in matched[0]
                    else ("driven" if "从动" in matched[0] else "unknown")
                ),
                "side_body": (
                    "thigh"
                    if "大腿" in matched[0]
                    else ("crank" if "小腿" in matched[0] else "unknown")
                ),
                **counted,
            }
        )
    entries.sort(key=lambda item: (item["role"], item["part"]))

    def find(role: str, body: str) -> dict[str, Any] | None:
        for entry in entries:
            if entry["role"] == role and entry["side_body"] == body:
                return entry
        return None

    drives: list[dict[str, Any]] = []
    for body in ("thigh", "crank"):
        driving, driven = find("driving", body), find("driven", body)
        record: dict[str, Any] = {"body": body}
        if not driving or not driven or not driving["teeth"] or not driven["teeth"]:
            record["status"] = "incomplete"
            record["driving_teeth"] = driving["teeth"] if driving else None
            record["driven_teeth"] = driven["teeth"] if driven else None
            drives.append(record)
            continue
        ratio = driven["teeth"] / driving["teeth"]
        same_pitch = (
            driving["standard_pitch_mm"] is not None
            and driving["standard_pitch_mm"] == driven["standard_pitch_mm"]
        )
        record.update(
            {
                "driving_teeth": driving["teeth"],
                "driven_teeth": driven["teeth"],
                "ratio": round(ratio, 6),
                "same_standard_pitch": same_pitch,
                "status": (
                    "confirmed" if same_pitch else "ratio_only_pitch_mismatch"
                ),
                "note": (
                    "driving sprocket on the motor, driven on the hip axis;"
                    " the driven side turns 1/ratio of the motor"
                ),
            }
        )
        drives.append(record)
    return {"sprockets": entries, "drives": drives}


def _gear_train(
    by_rep: dict[int, list[dict[str, Any]]],
    rep_names: dict[int, list[str]],
) -> dict[str, Any]:
    """Reduction ratio of the wheel gearbox, derived and then checked.

    The ratio is not an attribute anywhere in the file, so it has to be
    reconstructed.  Three independent facts are used, in this order:

    1. the tooth counts and module are in the part names
       (``STD_DualSpurGear_M0.6xT14_M0.6xT46``, ``STD_InternalSpurGear_M0.6xT77``);
    2. the measured cylinder radii must match the tip/root radius predicted by
       those numbers - otherwise the name is not trusted;
    3. a compound planetary needs the sun-planet and planet-ring centre
       distances to be equal, which *fixes* the sun tooth count that the sun
       part name does not carry.

    Everything stays ``candidate``: which element is fixed and which is the
    output is a mate, and this file has none.
    """
    gears: list[dict[str, Any]] = []
    radii_by_rep: dict[int, list[float]] = {
        rep_id: [group["max_radius_mm"] for group in _dominant_axes(cylinders, limit=6)]
        for rep_id, cylinders in by_rep.items()
    }
    for rep_id, cylinders in by_rep.items():
        names = rep_names.get(rep_id) or []
        teeth = _gear_teeth(names)
        if not teeth:
            continue
        measured_radii = radii_by_rep.get(rep_id) or []
        for entry in teeth:
            predicted = _predicted_radii(entry["module_mm"], entry["teeth"], entry["internal"])
            candidates = [predicted["tip_mm"], predicted["root_mm"]]
            best = min(
                (
                    (abs(radius - target), radius, target)
                    for radius in measured_radii
                    for target in candidates
                ),
                default=None,
            )
            gears.append(
                {
                    "rep": rep_id,
                    "part": names[0] if names else f"<rep {rep_id}>",
                    "module_mm": entry["module_mm"],
                    "teeth": entry["teeth"],
                    "internal": entry["internal"],
                    "predicted": {
                        key: round(value, 6) for key, value in predicted.items()
                    },
                    "closest_measured_radius_mm": None if best is None else best[1],
                    "matched_on": (
                        None
                        if best is None
                        else ("tip" if best[2] == predicted["tip_mm"] else "root")
                    ),
                    "radius_error_mm": None if best is None else round(best[0], 6),
                    # 0.05 mm is far below one tooth of module 0.6 (0.94 mm pitch)
                    "radius_confirmed": best is not None and best[0] <= 0.05,
                }
            )
    if not gears:
        return {
            "status": "no_gear_names",
            "gears": [],
            "gaps": [
                "no part name carries an M<module>xT<teeth> token; the ratio cannot be derived"
            ],
        }

    planet_pairs: dict[int, list[dict[str, Any]]] = {}
    for gear in gears:
        if not gear["internal"]:
            planet_pairs.setdefault(gear["rep"], []).append(gear)
    ring = next((gear for gear in gears if gear["internal"]), None)
    sun: dict[str, Any] | None = None
    for rep_id, names in rep_names.items():
        if not any(marker in name for name in names for marker in SUN_GEAR_MARKERS):
            continue
        if _gear_teeth(names):
            continue  # a named gear is handled above
        radii = radii_by_rep.get(rep_id) or []
        sun = {
            "rep": rep_id,
            "part": names[0] if names else f"<rep {rep_id}>",
            "measured_max_radius_mm": max(radii) if radii else None,
            "teeth_from_name": None,
        }
        break

    result: dict[str, Any] = {
        "gears": gears,
        "planet_reps": len(planet_pairs),
        "planet_teeth": sorted(
            {gear["teeth"] for gears_ in planet_pairs.values() for gear in gears_}
        ),
        "ring_teeth": ring["teeth"] if ring else None,
        "ring_part": ring["part"] if ring else None,
        "sun_part": sun["part"] if sun else None,
        "sun_teeth_from_name": sun.get("teeth_from_name") if sun else None,
        "module_mm": sorted({gear["module_mm"] for gear in gears}),
        "status": "candidate",
    }

    compound = [
        sorted(gears_, key=lambda item: item["teeth"])
        for gears_ in planet_pairs.values()
        if len({gear["teeth"] for gear in gears_}) > 1
    ]
    if not ring or not compound:
        result["status"] = "inconclusive"
        result["note"] = "need one internal gear and one compound (two tooth counts) planet"
        return result

    low, high = compound[0][0], compound[0][-1]
    result["compound_planet"] = {
        "small_teeth": low["teeth"],
        "large_teeth": high["teeth"],
        "instances": len(compound),
        "module_mm": low["module_mm"],
    }
    # equal centre distance: (Z_sun + Z_large)/2 = (Z_ring - Z_small)/2
    sun_teeth = ring["teeth"] - low["teeth"] - high["teeth"]
    if sun_teeth <= 0:
        result["status"] = "inconclusive"
        result["note"] = (
            f"the centre-distance constraint gives Z_sun = {sun_teeth}, which is not a gear; "
            "this is not a compound planetary with the ring fixed"
        )
        return result
    centre_planet_ring = high["module_mm"] * (ring["teeth"] - low["teeth"]) / 2.0
    centre_sun_planet = high["module_mm"] * (sun_teeth + high["teeth"]) / 2.0
    result["sun_teeth_derived"] = sun_teeth
    operator_confirmed_sun_teeth = CAD_CONFIRMED_SUN_TEETH
    result["sun_teeth_matches_cad"] = sun_teeth == operator_confirmed_sun_teeth
    result["centre_distance"] = {
        "planet_ring_mm": round(centre_planet_ring, 6),
        "sun_planet_mm": round(centre_sun_planet, 6),
        "equal": abs(centre_planet_ring - centre_sun_planet) < 1e-9,
    }
    if sun and sun["measured_max_radius_mm"] is not None:
        predicted_tip = _predicted_radii(high["module_mm"], sun_teeth, False)["tip_mm"]
        result["sun_radius_check"] = {
            "predicted_tip_mm": round(predicted_tip, 6),
            "measured_max_radius_mm": round(sun["measured_max_radius_mm"], 6),
            "relative_error": round(
                abs(sun["measured_max_radius_mm"] - predicted_tip) / predicted_tip, 6
            ),
            "note": "a corrected (profile shifted) pinion sits slightly above the "
            "standard tip radius; within ~2% supports the derived tooth count",
        }
    else:
        result["sun_radius_check"] = None
    def _ratio(sun_input: dict[str, Any], ring_mesh: dict[str, Any]) -> dict[str, float]:
        # ring fixed, carrier output -> 1 + (Zr/Zp_ring)*(Zp_sun/Zs)
        # carrier fixed, ring output ->    (Zr*Zp_sun)/(Zs*Zp_ring)
        carrier_out = 1.0 + (ring["teeth"] / ring_mesh["teeth"]) * (
            sun_input["teeth"] / sun_teeth
        )
        ring_out = (ring["teeth"] * sun_input["teeth"]) / (sun_teeth * ring_mesh["teeth"])
        return {
            "ring_fixed_carrier_output": round(carrier_out, 4),
            "carrier_fixed_ring_output": round(ring_out, 4),
        }

    result["ratio"] = {
        "formula_ring_fixed": "1 + (Z_ring/Z_planet_ring_side) * (Z_planet_sun_side/Z_sun)",
        "formula_carrier_fixed": "(Z_sun*Z_planet_sun_side)/(Z_ring*Z_planet_ring_side)"
        " -> reduction = 1/that",
        "sun_teeth": sun_teeth,
        "arrangements": [
            {
                "sun_meshes": high["teeth"],
                "ring_meshes": low["teeth"],
                "centre_distance_mm": round(centre_planet_ring, 6),
                **_ratio(high, low),
            },
            {
                "sun_meshes": low["teeth"],
                "ring_meshes": high["teeth"],
                "centre_distance_mm": round(
                    high["module_mm"] * (ring["teeth"] - low["teeth"]) / 2.0, 6
                ),
                **_ratio(low, high),
            },
        ],
        # 2026-10-31: the operator read the mates in SolidWorks - the carrier is
        # fixed and the ring gear is the output, so the ordinary-gear-train
        # arrangement (carrier fixed) is the real one, not the planetary one.
        "configuration": {
            "fixed": "carrier",
            "output": "ring",
            "selected": "carrier_fixed_ring_output",
            "reduction": _ratio(high, low)["carrier_fixed_ring_output"],
            "source": "operator read the mates in SolidWorks, 2026-10-31",
            "in_wheel_power_path": False,
            "why_not_in_path": (
                "operator: the wheel is driven directly by the M3508 geared motor,"
                " without this gearbox"
            ),
            "output_direction": "opposite to the sun: external mesh reverses once,"
            " the internal mesh keeps it",
        },
        "status": "confirmed_arrangement",
        "note": (
            "tooth counts and the fixed/output elements are both settled; the gearbox"
            " is nevertheless not in the wheel drive train"
        ),
    }
    result["gaps"] = [
        "which element is fixed (ring or carrier): settled 2026-10-31 (carrier fixed, ring output)",
        "sprocket tooth counts: counted from the tooth seats (16 / 19, 12.7 mm chain)",
        "efficiency: a two-mesh planetary is ~0.95-0.97 per mesh; not in the file",
        "whether this gearbox is in the machine at all: operator says the wheel is driven"
        " by the M3508's own reducer, so it is modelled but not used",
    ]
    return result


def _path_multiplicity(instances: Sequence[dict[str, Any]], name: str, limit: int = 10) -> dict[str, Any]:
    """How many physical copies of ``name`` the assembly actually contains.

    The file stores one NAUO per *product pair*, not per placement: the motor
    appears once as "MOTOR inside gearbox" even though the gearbox itself is
    instantiated twice.  The installed count is therefore the product of the
    multiplicities along the parent chain.
    """
    edges: dict[tuple[str, str], int] = {}
    for entry in instances:
        key = (entry["parent"], entry["child"])
        edges[key] = edges.get(key, 0) + 1
    count = 1
    node = name
    path: list[str] = []
    for _ in range(limit):
        parents = {edge[0] for edge in edges if edge[1] == node}
        if not parents:
            break
        if len(parents) > 1:
            return {"count": count, "path": path, "ambiguous": True}
        parent = next(iter(parents))
        if parent in path:
            break
        count *= edges[(parent, node)]
        path.append(parent)
        node = parent
    return {"count": count, "path": path, "ambiguous": False}


def _drive_train(
    instances: Sequence[dict[str, Any]],
    by_rep: dict[int, list[dict[str, Any]]],
    rep_names: dict[int, list[str]],
) -> dict[str, Any]:
    """Motor instances, transmission parts and the gas-spring geometry.

    What this can and cannot answer is the whole point of the section:

    * an *instance* (product, parent assembly, placement) is data the file has;
    * which instance is the hip joint and which is the knee joint is a mate,
      which SwSTEP does not export;
    * a cross-part distance is only meaningful when the two parts were exported
      in one datum, which is checked here per pair instead of assumed.
    """
    #: a product that is some other instance's parent is an assembly, not a part
    assembly_names = {entry["parent"] for entry in instances}
    motors: list[dict[str, Any]] = []
    excluded: list[dict[str, Any]] = []
    for entry in instances:
        name = entry["child"]
        if not any(pattern in name for pattern in DRIVE_MOTOR_PATTERNS):
            continue
        chain = _assembly_path(instances, name)
        is_assembly = name in assembly_names
        is_drivetrain_part = any(pattern in name for pattern in DRIVE_EXCLUDE_PATTERNS)
        in_module = any(DRIVE_MODULE_ASSEMBLY in node for node in chain["path"])
        record = {
            "product": name,
            "instance": entry["name"],
            "parent": entry["parent"],
            "assembly_path": chain["path"],
            "path_ambiguous": chain["ambiguous"],
            "translation_mm": (entry.get("transform_mm") or {}).get("translation_mm"),
            "rotation": (entry.get("transform_mm") or {}).get("rotation"),
            "rejected_because": (
                "sub-assembly"
                if is_assembly
                else (
                    "drivetrain part"
                    if is_drivetrain_part
                    else ("outside the wheel-leg module" if not in_module else None)
                )
            ),
        }
        keep = in_module and not is_assembly and not is_drivetrain_part
        (motors if keep else excluded).append(record)

    grouped: dict[str, dict[str, Any]] = {}
    for motor in motors:
        bucket = grouped.setdefault(
            motor["product"],
            {
                "product": motor["product"],
                "count": 0,
                "placements": [],
                "axis_candidates": [],
            },
        )
        bucket["placements"].append(
            {
                "instance": motor["instance"],
                "parent": motor["parent"],
                "translation_mm": motor["translation_mm"],
                "rotation": motor["rotation"],
                "path_ambiguous": motor["path_ambiguous"],
            }
        )
    for product, bucket in grouped.items():
        multiplicity = _path_multiplicity(instances, product)
        bucket["count"] = multiplicity["count"]
        bucket["install_parents"] = multiplicity["path"][:1]
        bucket["count_ambiguous"] = multiplicity["ambiguous"]
        bucket["axis_candidates"] = next(
            (
                part["dominant_axes"]
                for part in _part_axes(by_rep, rep_names, (product,))
                if part["dominant_axes"]
            ),
            [],
        )

    transmission: dict[tuple[str, str], int] = {}
    for entry in instances:
        if any(pattern in entry["child"] for pattern in DRIVE_TRANSMISSION_PATTERNS):
            key = (entry["child"], entry["parent"])
            transmission[key] = transmission.get(key, 0) + 1
    transmission_rows = [
        {"part": part, "parent": parent, "instances": count}
        for (part, parent), count in sorted(transmission.items())
    ]

    spring_parts = _part_axes(by_rep, rep_names, GAS_SPRING_PART_PATTERNS)
    clevis_parts = _part_axes(by_rep, rep_names, GAS_SPRING_CLEVIS_PATTERNS)
    body = next((part for part in spring_parts if "缸" in part["matched"]), None)
    rod = next((part for part in spring_parts if "杆" in part["matched"]), None)
    gas_spring: dict[str, Any] = {
        "parts": spring_parts,
        "clevis_parts": clevis_parts,
        "status": "no_geometry",
    }
    if body and rod and body["dominant_axes"] and rod["dominant_axes"]:
        body_axis = body["dominant_axes"][0]
        rod_axis = rod["dominant_axes"][0]
        coaxial = _line_distance(
            tuple(body_axis["origin_mm"]), tuple(body_axis["axis"]),
            tuple(rod_axis["origin_mm"]), tuple(rod_axis["axis"]),
        )
        # the two halves point at each other: 180 deg is as coaxial as 0 deg
        line_angle = min(coaxial["angle_deg"], 180.0 - coaxial["angle_deg"])
        pins = [
            part["dominant_axes"][0]
            for part in clevis_parts
            if part["dominant_axes"]
        ]
        pin_distance = None
        if len(pins) == 2:
            first, second = pins
            vector = _sub(tuple(second["origin_mm"]), tuple(first["origin_mm"]))
            axis = _unit(tuple(body_axis["axis"]))
            along = _dot(vector, axis)
            pin_distance = {
                "center_distance_mm": round(_norm(vector), 6),
                "along_spring_axis_mm": round(abs(along), 6),
                "signed_along_spring_axis_mm": round(along, 6),
                "perpendicular_mm": round(
                    math.sqrt(max(0.0, _norm(vector) ** 2 - along * along)), 6
                ),
                "pin_axes": [first["axis"], second["axis"]],
                "pin_origins_mm": [first["origin_mm"], second["origin_mm"]],
            }
        gas_spring = {
            "parts": spring_parts,
            "clevis_parts": clevis_parts,
            "body_axis": {
                "axis": body_axis["axis"],
                "origin_mm": body_axis["origin_mm"],
                "max_radius_mm": body_axis["max_radius_mm"],
                "coaxial_cylinders": body_axis["count"],
            },
            "rod_axis": {
                "axis": rod_axis["axis"],
                "origin_mm": rod_axis["origin_mm"],
                "max_radius_mm": rod_axis["max_radius_mm"],
                "coaxial_cylinders": rod_axis["count"],
            },
            "body_rod_line": {
                "perpendicular_mm": round(coaxial["distance_mm"], 6),
                "angle_deg": round(line_angle, 6),
            },
            "pin_to_pin": pin_distance,
            "status": (
                "candidate_same_datum"
                if pin_distance is not None
                and line_angle < 1.0
                and coaxial["distance_mm"] < 0.01
                else "inconclusive"
            ),
        }

    return {
        "motor_patterns": list(DRIVE_MOTOR_PATTERNS),
        "module_assembly": DRIVE_MODULE_ASSEMBLY,
        "motors": list(grouped.values()),
        "motor_count": sum(item["count"] for item in grouped.values()),
        "excluded_motors": excluded,
        "transmission_parts": transmission_rows,
        "gearbox": _gear_train(by_rep, rep_names),
        "sprocket_train": _sprocket_train(by_rep, rep_names),
        # 2026-10-31, operator: the wheel is turned by the M3508 geared motor
        # directly; the #NW_GEARBOX planetary is in the CAD but not in the power
        # path.  Recorded here because every downstream ratio depends on it.
        "wheel_drive": {
            "motor": "M3508 with its integrated reducer, mounted directly on the wheel",
            "gearbox_in_path": False,
            "ratio_source": "the M3508's own reducer (datasheet), not derivable from this file",
            "status": "confirmed_by_operator",
            "crank_reference": "ground",
            "crank_reference_evidence": (
                "both DM-8009 sit on 髋关节大铝件 (chassis frame); rotating the thigh"
                " swings thigh + shin, moving the shin alone turns one motor"
            ),
        },
        "gas_spring": gas_spring,
        "gaps": [
            "hip/knee role of each DM-8009 instance: it is a mate, not a placement",
            "wheel drive identity: the file has a hub-motor distribution board and an "
            "M3508+MKIV gearbox; only the assembly mates decide which one drives the wheel",
            "gear ratio: derived from the part names plus the measured radii, but which "
            "element is fixed and which is the output is a mate - see drive_train.gearbox",
            "gas spring force and stroke: the name 1BKB0.45-75 is a lead, the file has no "
            "PROPERTY_DEFINITION, MASS_DENSITY or force attribute",
            "installed spring length at the reference pose: this file is the stowed pose",
            "transform convention: confirm one cross-part distance in SolidWorks before "
            "promoting candidate_same_datum to verified",
        ],
        "provenance": "cad_extracted; every number here is geometry, never a measurement",
    }

# --------------------------------------------------------------- tiny vectors
Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Vec3, fallback: Vec3 = (0.0, 0.0, 1.0)) -> Vec3:
    length = _norm(a)
    return _scale(a, 1.0 / length) if length > 1e-12 else fallback


def _round(a: Vec3, digits: int = 9) -> list[float]:
    # 9 decimals of a millimetre is 1 pm: below the double-precision noise of the
    # STEP text, so lengths derived from these coordinates are not corrupted by
    # the report's own formatting.
    return [round(value, digits) + 0.0 for value in a]


def _perpendicular_basis(axis: Vec3) -> tuple[Vec3, Vec3]:
    helper: Vec3 = (1.0, 0.0, 0.0) if abs(axis[0]) < 0.9 else (0.0, 1.0, 0.0)
    e1 = _unit(_cross(axis, helper))
    e2 = _cross(axis, e1)
    return e1, e2


def _angle_deg(a: Vec3, b: Vec3) -> float:
    cosine = max(-1.0, min(1.0, _dot(_unit(a), _unit(b))))
    return math.degrees(math.acos(cosine))


def _line_distance(p1: Vec3, d1: Vec3, p2: Vec3, d2: Vec3) -> dict[str, float]:
    """Distance between two axes, with the parallel/skew distinction made explicit."""
    d1u, d2u = _unit(d1), _unit(d2)
    angle = _angle_deg(d1u, d2u)
    delta = _sub(p2, p1)
    if angle < 0.5 or angle > 179.5:
        perpendicular = _norm(_cross(delta, d1u))
        along = abs(_dot(delta, d1u))
        return {"distance_mm": perpendicular, "along_axis_mm": along, "angle_deg": angle}
    # skew lines: distance along the common normal
    normal = _cross(d1u, d2u)
    skew = abs(_dot(delta, _unit(normal)))
    return {"distance_mm": skew, "along_axis_mm": float("nan"), "angle_deg": angle}


# -------------------------------------------------------------------- entities
@dataclass
class StepEntity:
    id: int
    kind: str
    args: str

    def refs(self) -> list[int]:
        return [int(value) for value in re.findall(r"#(\d+)", self.args)]

    def strings(self) -> list[str]:
        return re.findall(r"'((?:[^']|'')*)'", self.args)


def split_top_level(text: str) -> list[str]:
    """Split a STEP argument list on commas that are not nested or quoted."""
    parts: list[str] = []
    depth = 0
    quoted = False
    current: list[str] = []
    index = 0
    while index < len(text):
        char = text[index]
        if quoted:
            current.append(char)
            if char == "'":
                if index + 1 < len(text) and text[index + 1] == "'":
                    current.append("'")
                    index += 1
                else:
                    quoted = False
        elif char == "'":
            quoted = True
            current.append(char)
        elif char == "(":
            depth += 1
            current.append(char)
        elif char == ")":
            depth -= 1
            current.append(char)
        elif char == "," and depth == 0:
            parts.append("".join(current).strip())
            current = []
        else:
            current.append(char)
        index += 1
    if current:
        parts.append("".join(current).strip())
    return parts


def decode_step_string(text: str) -> str:
    """Decode the ISO-10303-21 string escapes (``\\X2\\``/``\\X\\``/``''``).

    SwSTEP writes Chinese part names as UTF-16BE hex, so a plain read shows
    ``26\\X2\\82f1...``.  Without this every name match silently returns nothing.
    """

    def _x2(match: re.Match[str]) -> str:
        try:
            return bytes.fromhex(match.group(1)).decode("utf-16-be")
        except (ValueError, UnicodeDecodeError):
            return match.group(0)

    text = _X2.sub(_x2, text)
    text = _X1.sub(lambda match: chr(int(match.group(1), 16)), text)
    return text.replace("''", "'").replace("\\\\", "\\")


def _sha256(path: Path, chunk: int = 1 << 20) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            block = handle.read(chunk)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _entities(
    path: Path, encoding: str = "latin-1", chunk_size: int = 8 << 20
) -> tuple[dict[int, StepEntity], dict[int, dict[str, str]], dict[str, int]]:
    """Stream the DATA section; keep selected entities, count the rest.

    Complex instances (``#id = ( SUBTYPE(...) SUBTYPE(...) )``) carry several
    sub-types under one id, so they are returned separately: overwriting one
    with another drops the ``..._WITH_TRANSFORMATION`` half and makes every
    instance transform silently disappear.
    """
    kept: dict[int, StepEntity] = {}
    complex_kept: dict[int, dict[str, str]] = {}
    counts: dict[str, int] = {}
    carry = ""
    in_data = False
    with open(path, "r", encoding=encoding, errors="replace") as handle:
        while True:
            block = handle.read(chunk_size)
            if not block:
                break
            text = carry + block
            if not in_data:
                marker = text.find("DATA;")
                if marker < 0:
                    carry = text[-32:]
                    continue
                text = text[marker + 5 :]
                in_data = True
            statements = text.split(";")
            carry = statements.pop()  # may be an incomplete statement
            for statement in statements:
                match = _ENTITY.search(statement)
                if match:
                    kind = match.group(2)
                    if kind in COUNTED:
                        counts[kind] = counts.get(kind, 0) + 1
                        continue
                    if kind not in KEEP:
                        continue
                    end = statement.rfind(")")
                    args = statement[match.end() : end] if end > match.end() else ""
                    kept[int(match.group(1))] = StepEntity(int(match.group(1)), kind, args)
                    counts[kind] = counts.get(kind, 0) + 1
                    continue
                complex_match = _COMPLEX.search(statement)
                if not complex_match:
                    continue
                subtypes = _SUBTYPE.findall(statement[complex_match.end() :])
                payload = {subtype: args for subtype, args in subtypes if subtype in KEEP}
                if payload:
                    complex_kept[int(complex_match.group(1))] = payload
                for subtype, _args in subtypes:
                    if subtype in KEEP or subtype in COUNTED:
                        counts[subtype] = counts.get(subtype, 0) + 1
    return kept, complex_kept, counts


def _coordinates(entity: StepEntity | None) -> Vec3 | None:
    """``CARTESIAN_POINT(name, (x,y,z))`` / ``DIRECTION(name, (x,y,z))``."""
    if entity is None:
        return None
    args = split_top_level(entity.args)
    if not args:
        return None
    raw = args[-1].strip().strip("()")
    values = [float(value) for value in raw.split(",") if value.strip()]
    return (values[0], values[1], values[2]) if len(values) >= 3 else None


def _placement(
    entities: dict[int, StepEntity], placement_id: int
) -> tuple[Vec3, Mat3] | None:
    placement = entities.get(placement_id)
    if placement is None or placement.kind != "AXIS2_PLACEMENT_3D":
        return None
    refs = placement.refs()
    point = entities.get(refs[0]) if refs else None
    axis = entities.get(refs[1]) if len(refs) > 1 else None
    ref_dir = entities.get(refs[2]) if len(refs) > 2 else None
    origin = _coordinates(point)
    if origin is None:
        return None
    z_axis = _coordinates(axis) or (0.0, 0.0, 1.0)
    x_hint = _coordinates(ref_dir) or (1.0, 0.0, 0.0)
    z_axis = _unit(z_axis)
    x_axis = _sub(x_hint, _scale(z_axis, _dot(x_hint, z_axis)))
    x_axis = _unit(x_axis, (1.0, 0.0, 0.0))
    y_axis = _cross(z_axis, x_axis)
    # rows of the rotation matrix map local -> parent
    rotation: Mat3 = (
        (x_axis[0], y_axis[0], z_axis[0]),
        (x_axis[1], y_axis[1], z_axis[1]),
        (x_axis[2], y_axis[2], z_axis[2]),
    )
    return origin, rotation


def _mat_vec(matrix: Mat3, vector: Vec3) -> Vec3:
    return (
        _dot(matrix[0], vector),
        _dot(matrix[1], vector),
        _dot(matrix[2], vector),
    )


def _mat_transpose(matrix: Mat3) -> Mat3:
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def _mat_mul(a: Mat3, b: Mat3) -> Mat3:
    bt = _mat_transpose(b)
    return tuple(tuple(_dot(row, column) for column in bt) for row in a)  # type: ignore[return-value]


def _rigid_inverse(frame: tuple[Vec3, Mat3]) -> tuple[Vec3, Mat3]:
    origin, rotation = frame
    transposed = _mat_transpose(rotation)
    return _scale(_mat_vec(transposed, origin), -1.0), transposed


def _rigid_mul(
    a: tuple[Vec3, Mat3], b: tuple[Vec3, Mat3]
) -> tuple[Vec3, Mat3]:
    origin_a, rotation_a = a
    origin_b, rotation_b = b
    return _add(_mat_vec(rotation_a, origin_b), origin_a), _mat_mul(rotation_a, rotation_b)


# ------------------------------------------------------------------- geometry
def _cylinders_by_part(
    entities: dict[int, StepEntity],
    complex_entities: dict[int, dict[str, str]],
    definition_to_product: dict[int, int],
    formation_to_product: dict[int, int],
    products: dict[int, str],
    usage: dict[int, StepEntity],
) -> tuple[dict[str, list[dict[str, Any]]], dict[str, Any]]:
    """Attribute every ``CYLINDRICAL_SURFACE`` to the part that owns it."""
    solid_shells: dict[int, list[int]] = {}
    shell_faces: dict[int, list[int]] = {}
    face_surface: dict[int, int] = {}
    cylinders: dict[int, tuple[int, float]] = {}
    representations: dict[int, list[int]] = {}
    rep_name: dict[int, str] = {}
    for entity in entities.values():
        if entity.kind == "MANIFOLD_SOLID_BREP":
            refs = entity.refs()
            if refs:
                solid_shells[entity.id] = refs
        elif entity.kind == "CLOSED_SHELL":
            shell_faces[entity.id] = entity.refs()
        elif entity.kind == "ADVANCED_FACE":
            for ref in entity.refs():
                candidate = entities.get(ref)
                if candidate is not None and candidate.kind == "CYLINDRICAL_SURFACE":
                    face_surface[entity.id] = ref
                    break
        elif entity.kind == "CYLINDRICAL_SURFACE":
            args = split_top_level(entity.args)
            radius = float(args[-1]) if args else 0.0
            refs = entity.refs()
            if refs:
                cylinders[entity.id] = (refs[0], radius)
        elif entity.kind in ("SHAPE_REPRESENTATION", "ADVANCED_BREP_SHAPE_REPRESENTATION"):
            args = split_top_level(entity.args)
            names = entity.strings()
            rep_name[entity.id] = decode_step_string(names[0]) if names else ""
            refs = entity.refs()
            if refs:
                representations[entity.id] = refs

    def product_name(definition_id: int) -> str:
        formation = definition_to_product.get(definition_id)
        product = formation_to_product.get(formation, -1) if formation is not None else -1
        return products.get(product, "")

    # SHAPE_DEFINITION_REPRESENTATION(#product_definition_shape, #representation)
    rep_owner: dict[int, str] = {}
    shape_to_definition: dict[int, int] = {}
    for entity in entities.values():
        if entity.kind == "PRODUCT_DEFINITION_SHAPE":
            refs = entity.refs()
            if refs:
                shape_to_definition[entity.id] = refs[-1]
    for entity in entities.values():
        if entity.kind != "SHAPE_DEFINITION_REPRESENTATION":
            continue
        refs = entity.refs()
        if len(refs) < 2:
            continue
        definition = shape_to_definition.get(refs[0], refs[0])
        rep_id = refs[1]
        name = product_name(definition)
        if not name and definition in usage:
            usage_entity = usage[definition]
            usage_refs = usage_entity.refs()
            if len(usage_refs) >= 2:
                name = product_name(usage_refs[-1]) or (
                    decode_step_string((usage_entity.strings() or [""])[0])
                )
        if name:
            rep_owner[rep_id] = name

    # Attribute an owner through the *leaf part* representations only.
    #
    # A union over every representation relation merges the whole file into one
    # name (the 1004 relations form a single connected component).  Propagating
    # from the assembly rep downwards has the mirror problem: children inherit
    # the assembly name.  So the rule is: only a rep that belongs to a *leaf*
    # product (a product never used as a NAUO parent) may donate its name, only
    # one hop away, and an existing name is never overwritten.
    rep_candidates: dict[int, list[str]] = {}

    parents_of_usage = {
        usage_entity.refs()[-2]
        for usage_entity in usage.values()
        if len(usage_entity.refs()) >= 2
    }
    leaf_rep_names = {
        rep_id: name
        for rep_id, name in rep_owner.items()
        if rep_id not in parents_of_usage
    }
    adjacency: dict[int, set[int]] = {}
    for entity in entities.values():
        if entity.kind not in ("REPRESENTATION_RELATIONSHIP", "SHAPE_REPRESENTATION_RELATIONSHIP"):
            continue
        reps = [ref for ref in entity.refs() if ref in representations]
        if len(reps) == 2:
            adjacency.setdefault(reps[0], set()).add(reps[1])
            adjacency.setdefault(reps[1], set()).add(reps[0])
    for subtypes in complex_entities.values():
        args = subtypes.get("REPRESENTATION_RELATIONSHIP")
        if not args:
            continue
        reps = [int(value) for value in re.findall(r"#(\d+)", args) if int(value) in representations]
        if len(reps) == 2:
            adjacency.setdefault(reps[0], set()).add(reps[1])
            adjacency.setdefault(reps[1], set()).add(reps[0])
    for rep_id in representations:
        names: list[str] = []
        if rep_id in leaf_rep_names:
            names.append(leaf_rep_names[rep_id])
        for neighbour in sorted(adjacency.get(rep_id, ())):
            if neighbour in leaf_rep_names:
                names.append(leaf_rep_names[neighbour])
        unique = list(dict.fromkeys(names))
        if unique:
            rep_candidates[rep_id] = unique[:4]

    # ``CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`` -> NAUO child is a second,
    # independent source; merge it in without duplicating names.
    cdsr_definition: dict[int, int] = {}
    for relation in entities.values():
        if relation.kind != "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION":
            continue
        refs = relation.refs()
        if len(refs) < 2:
            continue
        cdsr_definition[refs[0]] = shape_to_definition.get(refs[1], refs[1])

    def add_candidate(reps: Iterable[int], definition: int) -> None:
        if definition not in usage:
            return
        usage_refs = usage[definition].refs()
        if len(usage_refs) < 2:
            return
        child_name = product_name(usage_refs[-1])
        if not child_name:
            return
        for rep_id in reps:
            bucket = rep_candidates.setdefault(rep_id, [])
            if child_name not in bucket:
                bucket.append(child_name)

    for relation_id, definition in cdsr_definition.items():
        relation_entity = entities.get(relation_id)
        if relation_entity is None:
            continue
        add_candidate(
            [ref for ref in relation_entity.refs() if ref in representations], definition
        )
    for relation_id, subtypes in complex_entities.items():
        definition = cdsr_definition.get(relation_id)
        args = subtypes.get("REPRESENTATION_RELATIONSHIP")
        if definition is None or not args:
            continue
        add_candidate(
            [int(value) for value in re.findall(r"#(\d+)", args) if int(value) in representations],
            definition,
        )
    for rep_id, subtypes in complex_entities.items():
        if "SHAPE_REPRESENTATION" in subtypes:
            rep_name.setdefault(rep_id, "")

    by_rep: dict[int, list[dict[str, Any]]] = {}
    attributed = 0
    for rep_id, items in representations.items():
        for item in items:
            solid = entities.get(item)
            if solid is None or solid.kind != "MANIFOLD_SOLID_BREP":
                continue
            for shell_id in solid_shells.get(solid.id, []):
                for face_id in shell_faces.get(shell_id, []):
                    surface_id = face_surface.get(face_id)
                    if surface_id is None or surface_id not in cylinders:
                        continue
                    placement_id, radius = cylinders[surface_id]
                    placed = _placement(entities, placement_id)
                    if placed is None:
                        continue
                    origin, rotation = placed
                    by_rep.setdefault(rep_id, []).append(
                        {
                            "cylinder": surface_id,
                            "radius_mm": round(radius, 6),
                            # AXIS2_PLACEMENT_3D: the local +Z is the axis
                            "origin_mm": _round(origin),
                            "axis": _round((rotation[0][2], rotation[1][2], rotation[2][2])),
                        }
                    )
                    attributed += 1
    diagnostics = {
        "representations": len(representations),
        "representations_named_directly": len(rep_owner),
        "representations_with_candidate_names": len(rep_candidates),
        "solids": len(solid_shells),
        "shells": len(shell_faces),
        "faces_with_cylinder": len(face_surface),
        "cylinders": len(cylinders),
        "attributed_cylinders": attributed,
    }
    return by_rep, rep_candidates, diagnostics


def _dominant_axes(cylinders: Sequence[dict[str, Any]], limit: int = 6) -> list[dict[str, Any]]:
    """Coaxial cylinder groups, largest bore first.

    A bearing seat appears as many coaxial cylinders with different radii; they
    must be grouped, otherwise the "largest cylinder" reports an arbitrary
    slice of the same bore.
    """
    groups: dict[tuple[Any, ...], dict[str, Any]] = {}
    for entry in cylinders:
        axis = _unit((entry["axis"][0], entry["axis"][1], entry["axis"][2]))
        e1, e2 = _perpendicular_basis(axis)
        origin = (entry["origin_mm"][0], entry["origin_mm"][1], entry["origin_mm"][2])
        key = (
            tuple(round(value, 2) + 0.0 for value in axis if abs(value) > 1e-9),
            round(_dot(origin, e1), 1),
            round(_dot(origin, e2), 1),
        )
        group = groups.setdefault(
            key,
            {
                "axis": _round(axis),
                "origin_mm": _round(origin),
                "max_radius_mm": 0.0,
                "cylinders": [],
            },
        )
        group["max_radius_mm"] = max(group["max_radius_mm"], entry["radius_mm"])
        group["cylinders"].append(entry["cylinder"])
    ordered = sorted(groups.values(), key=lambda item: item["max_radius_mm"], reverse=True)
    for group in ordered:
        group["count"] = len(group["cylinders"])
    return ordered[:limit]


def _measure_pair(
    pair: tuple[str, int, int],
    cylinder_index: dict[int, dict[str, Any]],
    rep_candidates: dict[int, list[str]],
) -> dict[str, Any]:
    label, first, second = pair
    entry_a = cylinder_index.get(first)
    entry_b = cylinder_index.get(second)
    if entry_a is None or entry_b is None:
        return {
            "label": label,
            "ids": [first, second],
            "status": "missing_entity",
            "note": "one or both cylinder entities were not found or not attributed to a part",
        }
    rep_a, rep_b = entry_a["rep"], entry_b["rep"]
    names_a = rep_candidates.get(rep_a, [])
    names_b = rep_candidates.get(rep_b, [])
    if rep_a != rep_b:
        return {
            "label": label,
            "ids": [first, second],
            "status": "unmeasured_cross_part",
            "reps": [rep_a, rep_b],
            "candidate_parts": [names_a[:3], names_b[:3]],
            "note": "cross-part distance depends on the unverified assembly transform convention",
        }
    geometry = _line_distance(
        (entry_a["origin_mm"][0], entry_a["origin_mm"][1], entry_a["origin_mm"][2]),
        (entry_a["axis"][0], entry_a["axis"][1], entry_a["axis"][2]),
        (entry_b["origin_mm"][0], entry_b["origin_mm"][1], entry_b["origin_mm"][2]),
        (entry_b["axis"][0], entry_b["axis"][1], entry_b["axis"][2]),
    )
    return {
        "label": label,
        "ids": [first, second],
        "rep": rep_a,
        "candidate_parts": names_a[:3],
        "status": "measured_same_part" if names_a else "measured_same_rep_unnamed",
        "distance_mm": round(geometry["distance_mm"], 9),
        "along_axis_mm": (
            round(geometry["along_axis_mm"], 9)
            if geometry["along_axis_mm"] == geometry["along_axis_mm"]
            else None
        ),
        "angle_deg": round(geometry["angle_deg"], 6),
        "radius_mm": [entry_a["radius_mm"], entry_b["radius_mm"]],
        "origins_mm": [entry_a["origin_mm"], entry_b["origin_mm"]],
        "axes": [entry_a["axis"], entry_b["axis"]],
        "provenance": "cad_extracted (cylinder axes, not a hardware measurement)",
    }


def _planar_candidates(measurements: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Hip / knee / wheel centres in the shared YZ plane, if the data supports it.

    Every leg part in this assembly rotates about the local X axis, so the
    mechanism is planar in YZ and the YZ coordinates of the joints can be
    compared *between* parts without resolving the (unverified) assembly
    transform.  The evidence that this is legitimate is the knee point: it is
    measured independently inside the thigh and inside the shin and must land on
    the same YZ coordinate.  If it does not, this function returns
    ``inconclusive`` instead of inventing a mechanism.
    """
    buckets: dict[tuple[float, float], list[dict[str, Any]]] = {}
    for entry in measurements:
        if not str(entry.get("status", "")).startswith("measured"):
            continue
        code = entry["label"].split(" ", 1)[0]
        if code not in ("G01", "G02"):
            continue
        origins = entry.get("origins_mm") or []
        radii = entry.get("radius_mm") or []
        for origin, radius in zip(origins, radii):
            key = (round(origin[1], 2), round(origin[2], 2))
            buckets.setdefault(key, []).append(
                {"code": code, "radius_mm": radius, "origin_mm": origin}
            )
    shared = {
        key: value
        for key, value in buckets.items()
        if len({item["code"] for item in value}) > 1
    }
    if len(shared) != 1:
        return {
            "status": "inconclusive",
            "reason": f"expected exactly one shared joint between G01 and G02, found {len(shared)}",
            "shared_points": {f"{k[0]},{k[1]}": v for k, v in shared.items()},
        }
    knee_key = next(iter(shared))
    others = [key for key in buckets if key != knee_key]
    if len(others) != 2:
        return {
            "status": "inconclusive",
            "reason": f"expected two non-shared points, found {len(others)}",
        }

    def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def full_point(key: tuple[float, float]) -> list[float]:
        """Use the unrounded origin: rounding to 0.01 mm would bias the lengths."""
        origin = buckets[key][0]["origin_mm"]
        return [origin[1], origin[2]]

    hip_key, wheel_key = others
    hip_point = full_point(hip_key)
    knee_point = full_point(knee_key)
    wheel_point = full_point(wheel_key)
    return {
        "status": "candidate",
        "plane": "YZ (every leg axis is along X; the X offset does not affect the planar geometry)",
        "points_mm": {
            "hip": hip_point,
            "knee": knee_point,
            "wheel": wheel_point,
        },
        "link_lengths_mm": {
            "thigh_hip_to_knee": round(distance(hip_point, knee_point), 9),
            "shin_knee_to_wheel": round(distance(knee_point, wheel_point), 9),
            "hip_to_wheel_chord": round(distance(hip_point, wheel_point), 9),
        },
        "evidence": (
            "the knee coordinate is measured independently inside the thigh (G01) and "
            "inside the shin (G02) and coincides to 0.01 mm"
        ),
        "caveats": [
            "the zero pose is the assembly pose, not a controller zero",
            "sign conventions and the direction of positive rotation are still unknown",
            "this is a 2-link + linkage projection; it is not yet the closed-chain constraint set",
        ],
    }


def _shared_axis_nodes(
    selected: Sequence[dict[str, Any]],
    tolerance_mm: float = 1e-3,
    min_parts: int = 2,
) -> dict[str, Any]:
    """Axis points that coincide across *different* parts.

    Two separate facts have to stay separate here:

    * a **cross-part distance** needs the assembly transform, whose convention is
      unverified - that is why :func:`_measure_pair` refuses it;
    * a **coincident point** between two parts needs no transform at all. If part
      A and part B both have a bore whose axis passes through the same ``(Y, Z)``
      to within a micron, then those two parts were exported in one datum and the
      coincidence is the joint between them.

    The result is evidence for building a planar mechanism graph. It is still
    ``cad_extracted``: coincidence identifies where a joint sits, not which body
    is rigidly attached to which - that remains a CAD question.
    """
    points: list[dict[str, Any]] = []
    for part in selected:
        names = part.get("candidate_parts") or [f"<rep {part.get('rep')}>"]
        # every coaxial group, not just the six largest: a Ø24 bearing seat in a
        # thigh plate sat behind the hip/knee bosses and was invisible
        for group in part.get("all_axes") or part.get("dominant_axes", []):
            origin = group["origin_mm"]
            points.append(
                {
                    "part": names[0],
                    "rep": part.get("rep"),
                    "y": float(origin[1]),
                    "z": float(origin[2]),
                    "radius_mm": float(group["max_radius_mm"]),
                    "coaxial_count": int(group["count"]),
                }
            )
    if not points:
        return {
            "status": "no_geometry",
            "tolerance_mm": tolerance_mm,
            "shared_nodes": [],
        }

    # Leader clustering: a grid/rounding key would split a node that straddles a
    # cell boundary, which is exactly the case this function exists to find.
    clusters: list[list[dict[str, Any]]] = []
    for point in points:
        for cluster in clusters:
            head = cluster[0]
            if math.hypot(point["y"] - head["y"], point["z"] - head["z"]) <= tolerance_mm:
                cluster.append(point)
                break
        else:
            clusters.append([point])

    nodes: list[dict[str, Any]] = []
    for cluster in clusters:
        by_part: dict[str, dict[str, Any]] = {}
        for point in cluster:
            entry = by_part.setdefault(
                point["part"],
                {"part": point["part"], "rep": point.get("rep"), "max_radius_mm": 0.0, "coaxial_count": 0},
            )
            entry["max_radius_mm"] = max(entry["max_radius_mm"], point["radius_mm"])
            entry["coaxial_count"] = max(entry["coaxial_count"], point["coaxial_count"])
        if len(by_part) < min_parts:
            continue
        ys = [point["y"] for point in cluster]
        zs = [point["z"] for point in cluster]
        spread = max(
            math.hypot(point["y"] - ys[0], point["z"] - zs[0]) for point in cluster
        )
        nodes.append(
            {
                "point_mm": [round(sum(ys) / len(ys), 9), round(sum(zs) / len(zs), 9)],
                "spread_mm": round(spread, 9),
                "part_count": len(by_part),
                "parts": sorted(
                    by_part.values(), key=lambda item: item["max_radius_mm"], reverse=True
                ),
            }
        )
    nodes.sort(
        key=lambda item: (
            -item["part_count"],
            -max(part["max_radius_mm"] for part in item["parts"]),
        )
    )
    return {
        "status": "shared_datum_candidates",
        "tolerance_mm": tolerance_mm,
        "min_parts": min_parts,
        "points_considered": len(points),
        "clusters": len(clusters),
        "shared_nodes": nodes,
        "evidence": (
            "parts whose bore axes coincide to within the tolerance were exported in one "
            "coordinate datum; the coincidence locates a joint without needing the assembly "
            "transform"
        ),
        "caveats": [
            "a shared point says where a joint is, not which body is rigidly attached to which",
            "parts that do not appear here may still be in another datum; absence is not evidence",
            "coincident coordinates are cad_extracted, never a hardware measurement",
        ],
    }


# --------------------------------------------------------------------- public
def audit_step(
    path: str | Path,
    encoding: str = "latin-1",
    keep_products: int = 400,
    axis_pairs: Sequence[tuple[str, int, int]] = DEFAULT_AXIS_PAIRS,
    cross_part_pairs: Sequence[tuple[str, int, int]] = DEFAULT_CROSS_PART_PAIRS,
    interesting_parts: Sequence[str] = INTERESTING_PARTS,
    with_geometry: bool = True,
) -> dict[str, Any]:
    """Return the structure + axis audit for one STEP file.

    ``encoding`` stays ``latin-1`` on purpose: the DATA section is ASCII with
    ``\\X2\\`` escapes, and latin-1 cannot fail on any byte, so a stray raw GBK
    byte in the header cannot abort a 264 MB parse.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    with open(path, "r", encoding="latin-1", errors="replace") as handle:
        head = handle.read(8192)
    header = {
        "file_name": _header_value(head, "FILE_NAME", 0),
        "file_description": _header_value(head, "FILE_DESCRIPTION", 0),
        "file_schema": _header_value(head, "FILE_SCHEMA", 0),
        "exporter": _header_value(head, "FILE_NAME", 4),
        "author": _header_value(head, "FILE_NAME", 2),
    }
    entities, complex_entities, counts = _entities(path, encoding=encoding)

    products: dict[int, str] = {}
    formation_to_product: dict[int, int] = {}
    definition_to_product: dict[int, int] = {}
    for entity in entities.values():
        if entity.kind == "PRODUCT":
            names = entity.strings()
            products[entity.id] = decode_step_string(names[0]) if names else ""
        elif entity.kind in (
            "PRODUCT_DEFINITION_FORMATION",
            "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE",
        ):
            refs = entity.refs()
            if refs:
                formation_to_product[entity.id] = refs[0]
        elif entity.kind == "PRODUCT_DEFINITION":
            refs = entity.refs()
            if refs:
                definition_to_product[entity.id] = refs[0]

    def product_name(definition_id: int) -> str:
        formation = definition_to_product.get(definition_id)
        product = formation_to_product.get(formation, -1) if formation is not None else -1
        return products.get(product, f"<product-of-definition-{definition_id}>")

    usage = {
        entity.id: entity
        for entity in entities.values()
        if entity.kind == "NEXT_ASSEMBLY_USAGE_OCCURRENCE"
    }
    transforms = {
        entity.id: entity
        for entity in entities.values()
        if entity.kind == "ITEM_DEFINED_TRANSFORMATION"
    }
    shape_to_definition: dict[int, int] = {}
    for entity in entities.values():
        if entity.kind == "PRODUCT_DEFINITION_SHAPE":
            refs = entity.refs()
            if refs:
                shape_to_definition[entity.id] = refs[-1]
    relation_to_transform: dict[int, int] = {}
    for entity in entities.values():
        if entity.kind in (
            "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
            "SHAPE_REPRESENTATION_RELATIONSHIP",
        ):
            refs = entity.refs()
            if refs:
                relation_to_transform[entity.id] = refs[0]
    for relation_id, subtypes in complex_entities.items():
        args = subtypes.get("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
        if args is None:
            continue
        refs = [int(value) for value in re.findall(r"#(\d+)", args)]
        if refs:
            relation_to_transform[relation_id] = refs[0]

    usage_transform: dict[int, int] = {}
    for relation in entities.values():
        if relation.kind != "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION":
            continue
        refs = relation.refs()
        if len(refs) < 2:
            continue
        transform_id = relation_to_transform.get(refs[0])
        usage_id = shape_to_definition.get(refs[1], refs[1])
        if transform_id is not None and transform_id in transforms and usage_id in usage:
            usage_transform[usage_id] = transform_id

    instances: list[dict[str, Any]] = []
    children: set[int] = set()
    for usage_id, usage_entity in usage.items():
        refs = usage_entity.refs()
        if len(refs) < 2:
            continue
        parent_pd, child_pd = refs[-2], refs[-1]
        children.add(child_pd)
        entry: dict[str, Any] = {
            "usage_id": usage_id,
            "name": decode_step_string((usage_entity.strings() or [""])[0]),
            "parent": product_name(parent_pd),
            "child": product_name(child_pd),
        }
        transform_id = usage_transform.get(usage_id)
        if transform_id is not None:
            transform = transforms.get(transform_id)
            refs_t = transform.refs() if transform is not None else []
            if len(refs_t) >= 2:
                parent_frame = _placement(entities, refs_t[0])
                child_frame = _placement(entities, refs_t[1])
                if parent_frame and child_frame:
                    # STEP stores the two ends of the relation; the instance
                    # transform is the child placement expressed in the parent.
                    matrix = _rigid_mul(parent_frame, _rigid_inverse(child_frame))
                    entry["transform_mm"] = {
                        "translation_mm": _round(matrix[0]),
                        "rotation": [_round(row, 9) for row in matrix[1]],
                        "convention": "A_axis1 * inv(A_axis2), unverified",
                    }
        instances.append(entry)

    roots = sorted(
        {
            definition
            for definition in definition_to_product
            if definition not in children and product_name(definition) not in ("", "None")
        }
    )
    product_counts: dict[str, int] = {}
    for entry in instances:
        product_counts[entry["child"]] = product_counts.get(entry["child"], 0) + 1

    units = [
        {"id": entity.id, "kind": entity.kind, "args": entity.args[:160]}
        for entity in entities.values()
        if entity.kind in ("SI_UNIT", "CONVERSION_BASED_UNIT") and "METRE" in entity.args.upper()
    ]
    for entity_id, subtypes in complex_entities.items():
        text = " ".join(subtypes.values()).upper()
        if "METRE" in text:
            units.append(
                {
                    "id": entity_id,
                    "kind": "+".join(subtypes),
                    "args": " ".join(f"{k}({v.strip()})" for k, v in subtypes.items())[:160],
                }
            )

    report: dict[str, Any] = {
        "stage": "S2-step-audit",
        "status": "structure_and_axis_candidates" if with_geometry else "structure_only",
        "source": {
            "path": str(path),
            "bytes": path.stat().st_size,
            "sha256": _sha256(path),
            "encoding": encoding,
            "header": header,
        },
        "units": {
            "length": "millimetre (converted to metre before simulation)",
            "records": units[:8],
        },
        "counts": {
            **counts,
            "PRODUCT": len(products),
            "PRODUCT_DEFINITION": len(definition_to_product),
            "NEXT_ASSEMBLY_USAGE_OCCURRENCE": len(usage),
            "ITEM_DEFINED_TRANSFORMATION": len(transforms),
        },
        "assembly": {
            "roots": [product_name(definition) for definition in roots][:20],
            "root_count": len(roots),
            "products": [
                {"name": name, "instances": product_counts.get(name, 0)}
                for name in list(dict.fromkeys(products.values()))[:keep_products]
            ],
        },
        "instances": instances,
        "instance_count": len(instances),
        "verification": {
            "mass_and_density_in_step": "not found in the previous audit; re-check before use",
            "mates_and_motion_axes": "not exported by SwSTEP; must be read in SolidWorks",
            "transform_convention": "unverified - compare at least two known parts in CAD",
            "same_part_distance": "convention-free: both cylinders live in one part frame",
            "provenance": "cad_extracted, never promoted to measured without hardware data",
        },
    }

    if with_geometry:
        by_rep, rep_candidates_parts, diagnostics = _cylinders_by_part(
            entities, complex_entities, definition_to_product, formation_to_product, products, usage
        )
        cylinder_index: dict[int, dict[str, Any]] = {}
        for rep_id, entries in by_rep.items():
            for entry in entries:
                cylinder_index[entry["cylinder"]] = {**entry, "rep": rep_id}
        selected = []
        for rep_id, entries in by_rep.items():
            names = rep_candidates_parts.get(rep_id, [])
            if not any(
                keyword in name
                for name in names
                for keyword in interesting_parts
            ):
                continue
            selected.append(
                {
                    "rep": rep_id,
                    "candidate_parts": names[:3],
                    "cylinder_count": len(entries),
                    "dominant_axes": _dominant_axes(entries),
                    "all_axes": _dominant_axes(entries, limit=len(entries)),
                }
            )
        selected.sort(key=lambda item: item["cylinder_count"], reverse=True)
        measurements = [
            _measure_pair(pair, cylinder_index, rep_candidates_parts) for pair in axis_pairs
        ]
        cross_part = [
            _measure_pair(pair, cylinder_index, rep_candidates_parts) for pair in cross_part_pairs
        ]
        shared = _shared_axis_nodes(selected)
        report["geometry"] = {
            "diagnostics": diagnostics,
            "reps_with_cylinders": len(by_rep),
            "parts": [                {
                    "rep": rep_id,
                    "cylinder_count": len(entries),
                    "candidate_parts": rep_candidates_parts.get(rep_id, [])[:3],
                }
                for rep_id, entries in sorted(
                    by_rep.items(), key=lambda item: len(item[1]), reverse=True
                )
            ][:80],
            "selected_parts": selected,
            "mechanism_candidates": _planar_candidates(measurements),
            "shared_axis_nodes": shared,
            "measurements": measurements,
            "cross_part": cross_part,
            "pin_carriers": _pin_carriers(by_rep, rep_candidates_parts),
            "verification": {
                "axis_definition": "AXIS2_PLACEMENT_3D local +Z is the cylinder axis",
                "distance": "perpendicular distance between two parallel axes, mm",
                "expected_g01_mm": 210.000,
                "expected_g02_mm": 250.000,
                "note": "reproducing progress.md §6.3 G01/G02 is this parser's acceptance test",
            },
        }
        report["drive_train"] = _drive_train(instances, by_rep, rep_candidates_parts)

    report["next_steps"] = [
        "confirm which DM-8009 instance is the hip joint and which is the knee joint in SolidWorks",
        "confirm the wheel drive (M3508 + MKIV gearbox or hub motor) from the assembly mates",
        "confirm one cross-part distance in SolidWorks before trusting candidate_same_datum",
        "read the MKIV gearbox ratio and the 1BKB0.45-75 force/stroke from their specifications",
        "record confirmed values in docs/STEP_GEOMETRY_AUDIT.md and configs/mechanism.yaml",
    ]
    return report


def _header_value(head: str, name: str, index: int) -> str | None:
    match = re.search(rf"{name}\s*\((.*?)\);", head, re.DOTALL)
    if not match:
        return None
    fields = split_top_level(match.group(1))
    if len(fields) <= index:
        return None
    return _decode_header(fields[index])


def _decode_header(text: str) -> str:
    """Header strings may contain raw non-UTF-8 bytes (GBK part names)."""
    stripped = text.strip()
    if len(stripped) >= 2 and stripped.startswith("'") and stripped.endswith("'"):
        stripped = stripped[1:-1]
    decoded = decode_step_string(stripped)
    try:
        return decoded.encode("latin-1").decode("gbk")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return decoded


def name_instances(report: dict[str, Any], needle: str, limit: int = 40) -> list[dict[str, Any]]:
    """Instances whose parent or child name contains ``needle`` (case-insensitive)."""
    needle = needle.lower()
    return [
        entry
        for entry in report.get("instances", [])
        if needle in str(entry.get("parent", "")).lower()
        or needle in str(entry.get("child", "")).lower()
    ][:limit]


def summarize(
    report: dict[str, Any], needles: Iterable[str] = ("髋", "hip", "spring", "弹簧", "DM8009")
) -> dict[str, Any]:
    summary: dict[str, Any] = {"counts": report["counts"], "roots": report["assembly"]["roots"]}
    summary["matches"] = {needle: name_instances(report, needle, limit=8) for needle in needles}
    drive = report.get("drive_train")
    if drive:
        spring = drive.get("gas_spring") or {}
        summary["drive_train"] = {
            "motors": drive.get("motor_count"),
            "excluded_name_matches": len(drive.get("excluded_motors") or []),
            "transmission_parts": len(drive.get("transmission_parts") or []),
            "gas_spring_status": spring.get("status"),
            "gas_spring_pin_to_pin_mm": (spring.get("pin_to_pin") or {}).get(
                "center_distance_mm"
            ),
        }
    return summary

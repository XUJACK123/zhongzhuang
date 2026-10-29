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


def _round(a: Vec3, digits: int = 6) -> list[float]:
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
        "distance_mm": round(geometry["distance_mm"], 6),
        "along_axis_mm": (
            round(geometry["along_axis_mm"], 6)
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
            "thigh_hip_to_knee": round(distance(hip_point, knee_point), 6),
            "shin_knee_to_wheel": round(distance(knee_point, wheel_point), 6),
            "hip_to_wheel_chord": round(distance(hip_point, wheel_point), 6),
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
                }
            )
        selected.sort(key=lambda item: item["cylinder_count"], reverse=True)
        measurements = [
            _measure_pair(pair, cylinder_index, rep_candidates_parts) for pair in axis_pairs
        ]
        cross_part = [
            _measure_pair(pair, cylinder_index, rep_candidates_parts) for pair in cross_part_pairs
        ]
        report["geometry"] = {
            "diagnostics": diagnostics,
            "reps_with_cylinders": len(by_rep),
            "parts": [
                {
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
            "measurements": measurements,
            "cross_part": cross_part,
            "verification": {
                "axis_definition": "AXIS2_PLACEMENT_3D local +Z is the cylinder axis",
                "distance": "perpendicular distance between two parallel axes, mm",
                "expected_g01_mm": 210.000,
                "expected_g02_mm": 250.000,
                "note": "reproducing progress.md §6.3 G01/G02 is this parser's acceptance test",
            },
        }

    report["next_steps"] = [
        "confirm the transform convention on two known parts",
        "map the hip sub-assembly to the four DM8009 instances",
        "confirm which dominant axes are the real hip / knee / wheel joints",
        "record results in docs/STEP_GEOMETRY_AUDIT.md and configs/mechanism.yaml",
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
    return summary

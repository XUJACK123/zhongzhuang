"""Minimal URDF reader / writer.

Only the subset used by this project is supported.  The reader keeps enough
information to rebuild a byte-deterministic URDF from a parsed model, which is
what ``assets/build_corrected.py`` needs in order to rewrite visual origins,
centres of mass and inertia tensors without touching the original files.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..math3d import Transform, matrix_to_sym_6, rpy_to_mat, sym_6_to_matrix

MESH_PREFIX = "package://"

#: Package name used when writing derived URDF files.  The reader strips the
#: first path segment as the package name, so the writer must emit one.
OUTPUT_PACKAGE = "fifty_mujoco"


def _fmt(value: float) -> str:
    """Shortest round-trip float formatting (``repr`` of a float64)."""
    text = repr(float(value))
    return "0" if text == "-0.0" else text


def _fmt_vec(values) -> str:
    return " ".join(_fmt(v) for v in values)


@dataclass
class Inertial:
    mass: float
    com: np.ndarray
    rpy: np.ndarray
    inertia: np.ndarray  # 3x3, about the COM, expressed in the inertial frame

    @property
    def inertia_link_frame(self) -> np.ndarray:
        """Inertia re-expressed in the link frame (single rotation)."""
        R = rpy_to_mat(self.rpy)
        return R @ self.inertia @ R.T

    def copy(self) -> "Inertial":
        return Inertial(float(self.mass), self.com.copy(), self.rpy.copy(), self.inertia.copy())


@dataclass
class Geometry:
    kind: str  # mesh | box | cylinder | sphere | capsule
    mesh_file: str | None = None
    size: np.ndarray | None = None  # box: (3,), cylinder/capsule: (radius, length), sphere: (radius,)
    scale: np.ndarray | None = None

    def copy(self) -> "Geometry":
        return Geometry(
            self.kind,
            self.mesh_file,
            None if self.size is None else self.size.copy(),
            None if self.scale is None else self.scale.copy(),
        )


@dataclass
class Shape:
    geometry: Geometry
    xyz: np.ndarray
    rpy: np.ndarray
    rgba: tuple[float, float, float, float] | None = None

    def transform(self) -> Transform:
        return Transform.from_xyz_rpy(self.xyz, self.rpy)

    def copy(self) -> "Shape":
        return Shape(
            self.geometry.copy(),
            self.xyz.copy(),
            self.rpy.copy(),
            self.rgba,
        )


@dataclass
class Link:
    name: str
    inertial: Inertial | None
    visuals: list[Shape] = field(default_factory=list)
    collisions: list[Shape] = field(default_factory=list)


@dataclass
class Joint:
    name: str
    type: str  # revolute | continuous | prismatic | fixed
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    damping: float | None = None
    frictionloss: float | None = None

    def origin(self) -> Transform:
        return Transform.from_xyz_rpy(self.xyz, self.rpy)

    @property
    def is_movable(self) -> bool:
        return self.type in {"revolute", "continuous", "prismatic"}


@dataclass
class RobotModel:
    name: str
    links: dict[str, Link]
    joints: dict[str, Joint]
    root_link: str

    # ------------------------------------------------------------------ queries
    def joint_list(self) -> list[Joint]:
        """Joints in declaration order."""
        return list(self.joints.values())

    def movable_joints(self) -> list[Joint]:
        return [j for j in self.joints.values() if j.is_movable]

    def child_map(self) -> dict[str, list[Joint]]:
        children: dict[str, list[Joint]] = {name: [] for name in self.links}
        for joint in self.joints.values():
            children.setdefault(joint.parent, []).append(joint)
        return children

    def total_mass(self) -> float:
        return float(sum(link.inertial.mass for link in self.links.values() if link.inertial))

    def copy(self) -> "RobotModel":
        return RobotModel(
            self.name,
            {
                name: Link(
                    name,
                    None if link.inertial is None else link.inertial.copy(),
                    [s.copy() for s in link.visuals],
                    [s.copy() for s in link.collisions],
                )
                for name, link in self.links.items()
            },
            {
                name: Joint(
                    j.name, j.type, j.parent, j.child, j.xyz.copy(), j.rpy.copy(), j.axis.copy(),
                    j.lower, j.upper, j.effort, j.velocity, j.damping, j.frictionloss,
                )
                for name, j in self.joints.items()
            },
            self.root_link,
        )


# --------------------------------------------------------------------------- parse
def _text_vec(element: ET.Element | None, tag: str, default: str) -> np.ndarray:
    if element is None:
        return np.array([float(v) for v in default.split()], dtype=float)
    raw = element.get(tag)
    if raw is None:
        return np.array([float(v) for v in default.split()], dtype=float)
    return np.array([float(v) for v in raw.split()], dtype=float)


def _parse_geometry(element: ET.Element) -> Geometry:
    mesh = element.find("mesh")
    if mesh is not None:
        filename = mesh.get("filename", "")
        if filename.startswith(MESH_PREFIX):
            filename = filename[len(MESH_PREFIX):]
        if "/" in filename:
            filename = filename.split("/", 1)[1]
        scale = mesh.get("scale")
        return Geometry(
            "mesh",
            mesh_file=filename,
            scale=None if scale is None else np.array([float(v) for v in scale.split()]),
        )
    box = element.find("box")
    if box is not None:
        return Geometry("box", size=np.array([float(v) for v in box.get("size", "0 0 0").split()]))
    cylinder = element.find("cylinder")
    if cylinder is not None:
        return Geometry(
            "cylinder",
            size=np.array([float(cylinder.get("radius", "0")), float(cylinder.get("length", "0"))]),
        )
    capsule = element.find("capsule")
    if capsule is not None:
        return Geometry(
            "capsule",
            size=np.array([float(capsule.get("radius", "0")), float(capsule.get("length", "0"))]),
        )
    sphere = element.find("sphere")
    if sphere is not None:
        return Geometry("sphere", size=np.array([float(sphere.get("radius", "0"))]))
    raise ValueError("unsupported or empty URDF geometry")


def _parse_shapes(link_element: ET.Element, tag: str) -> list[Shape]:
    shapes: list[Shape] = []
    for element in link_element.findall(tag):
        origin = element.find("origin")
        geometry_element = element.find("geometry")
        if geometry_element is None:
            continue
        rgba = None
        material = element.find("material")
        if material is not None:
            color = material.find("color")
            if color is not None and color.get("rgba"):
                rgba = tuple(float(v) for v in color.get("rgba").split())
        shapes.append(
            Shape(
                _parse_geometry(geometry_element),
                _text_vec(origin, "xyz", "0 0 0"),
                _text_vec(origin, "rpy", "0 0 0"),
                rgba,
            )
        )
    return shapes


def load_urdf(path: str | Path) -> RobotModel:
    path = Path(path)
    tree = ET.parse(path)
    root = tree.getroot()
    links: dict[str, Link] = {}
    for element in root.findall("link"):
        name = element.get("name", "")
        inertial = None
        inertial_element = element.find("inertial")
        if inertial_element is not None:
            origin = inertial_element.find("origin")
            mass_element = inertial_element.find("mass")
            inertia_element = inertial_element.find("inertia")
            mass = float(mass_element.get("value", "0")) if mass_element is not None else 0.0
            inertia_values = dict(inertia_element.attrib) if inertia_element is not None else {}
            inertial = Inertial(
                mass=mass,
                com=_text_vec(origin, "xyz", "0 0 0"),
                rpy=_text_vec(origin, "rpy", "0 0 0"),
                inertia=sym_6_to_matrix(inertia_values),
            )
        links[name] = Link(
            name,
            inertial,
            _parse_shapes(element, "visual"),
            _parse_shapes(element, "collision"),
        )

    joints: dict[str, Joint] = {}
    for element in root.findall("joint"):
        name = element.get("name", "")
        joint_type = element.get("type", "fixed")
        origin = element.find("origin")
        parent = element.find("parent")
        child = element.find("child")
        axis_element = element.find("axis")
        limit = element.find("limit")
        dynamics = element.find("dynamics")
        joints[name] = Joint(
            name=name,
            type=joint_type,
            parent=parent.get("link", "") if parent is not None else "",
            child=child.get("link", "") if child is not None else "",
            xyz=_text_vec(origin, "xyz", "0 0 0"),
            rpy=_text_vec(origin, "rpy", "0 0 0"),
            axis=_text_vec(axis_element, "xyz", "0 0 1"),
            lower=float(limit.get("lower")) if limit is not None and limit.get("lower") is not None else None,
            upper=float(limit.get("upper")) if limit is not None and limit.get("upper") is not None else None,
            effort=float(limit.get("effort")) if limit is not None and limit.get("effort") is not None else None,
            velocity=float(limit.get("velocity")) if limit is not None and limit.get("velocity") is not None else None,
            damping=float(dynamics.get("damping")) if dynamics is not None and dynamics.get("damping") is not None else None,
            frictionloss=float(dynamics.get("frictionloss")) if dynamics is not None and dynamics.get("frictionloss") is not None else None,
        )

    child_links = {j.child for j in joints.values()}
    roots = [name for name in links if name not in child_links]
    if len(roots) != 1:
        raise ValueError(f"{path}: expected exactly one root link, found {roots}")
    return RobotModel(root.get("name", path.stem), links, joints, roots[0])


# --------------------------------------------------------------------------- write
def _add_origin(parent: ET.Element, xyz, rpy) -> None:
    ET.SubElement(parent, "origin", {"xyz": _fmt_vec(xyz), "rpy": _fmt_vec(rpy)})


def _add_geometry(parent: ET.Element, geometry: Geometry) -> None:
    element = ET.SubElement(parent, "geometry")
    if geometry.kind == "mesh":
        attributes = {"filename": f"{MESH_PREFIX}{OUTPUT_PACKAGE}/{geometry.mesh_file}"}
        if geometry.scale is not None:
            attributes["scale"] = _fmt_vec(geometry.scale)
        ET.SubElement(element, "mesh", attributes)
    elif geometry.kind == "box":
        ET.SubElement(element, "box", {"size": _fmt_vec(geometry.size)})
    elif geometry.kind == "cylinder":
        ET.SubElement(
            element,
            "cylinder",
            {"radius": _fmt(geometry.size[0]), "length": _fmt(geometry.size[1])},
        )
    elif geometry.kind == "capsule":
        ET.SubElement(
            element,
            "capsule",
            {"radius": _fmt(geometry.size[0]), "length": _fmt(geometry.size[1])},
        )
    elif geometry.kind == "sphere":
        ET.SubElement(element, "sphere", {"radius": _fmt(geometry.size[0])})
    else:
        raise ValueError(f"unsupported geometry kind {geometry.kind!r}")


def dump_urdf(robot: RobotModel) -> ET.ElementTree:
    root = ET.Element("robot", {"name": robot.name})

    colors: dict[str, str] = {}
    for link in robot.links.values():
        for shape in link.visuals:
            if shape.rgba is None:
                continue
            key = _fmt_vec(shape.rgba)
            colors.setdefault(key, f"color_{len(colors)}")
    for key, material_name in colors.items():
        material = ET.SubElement(root, "material", {"name": material_name})
        ET.SubElement(material, "color", {"rgba": key})

    for link in robot.links.values():
        element = ET.SubElement(root, "link", {"name": link.name})
        if link.inertial is not None:
            inertial = ET.SubElement(element, "inertial")
            _add_origin(inertial, link.inertial.com, link.inertial.rpy)
            ET.SubElement(inertial, "mass", {"value": _fmt(link.inertial.mass)})
            ET.SubElement(inertial, "inertia", {k: _fmt(v) for k, v in matrix_to_sym_6(link.inertial.inertia).items()})
        for shape in link.visuals:
            visual = ET.SubElement(element, "visual")
            _add_origin(visual, shape.xyz, shape.rpy)
            _add_geometry(visual, shape.geometry)
            if shape.rgba is not None:
                ET.SubElement(visual, "material", {"name": colors[_fmt_vec(shape.rgba)]})
        for shape in link.collisions:
            collision = ET.SubElement(element, "collision")
            _add_origin(collision, shape.xyz, shape.rpy)
            _add_geometry(collision, shape.geometry)
            if shape.rgba is not None:
                material = ET.SubElement(collision, "material", {"name": f"collision_{link.name}"})
                ET.SubElement(material, "color", {"rgba": "0.85 0.35 0.25 0.35"})

    for joint in robot.joints.values():
        attributes = {"name": joint.name, "type": joint.type}
        element = ET.SubElement(root, "joint", attributes)
        _add_origin(element, joint.xyz, joint.rpy)
        ET.SubElement(element, "parent", {"link": joint.parent})
        ET.SubElement(element, "child", {"link": joint.child})
        if joint.is_movable:
            ET.SubElement(element, "axis", {"xyz": _fmt_vec(joint.axis)})
        limit_attributes: dict[str, str] = {}
        if joint.lower is not None and joint.upper is not None:
            limit_attributes["lower"] = _fmt(joint.lower)
            limit_attributes["upper"] = _fmt(joint.upper)
        if joint.effort is not None:
            limit_attributes["effort"] = _fmt(joint.effort)
        if joint.velocity is not None:
            limit_attributes["velocity"] = _fmt(joint.velocity)
        if limit_attributes:
            ET.SubElement(element, "limit", limit_attributes)
        dynamics_attributes: dict[str, str] = {}
        if joint.damping is not None:
            dynamics_attributes["damping"] = _fmt(joint.damping)
        if joint.frictionloss is not None:
            dynamics_attributes["frictionloss"] = _fmt(joint.frictionloss)
        if dynamics_attributes:
            ET.SubElement(element, "dynamics", dynamics_attributes)

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    return tree


def save_urdf(robot: RobotModel, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tree = dump_urdf(robot)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path

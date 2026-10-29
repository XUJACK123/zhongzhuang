"""P1: emit the two MJCF variants that share one set of physical parameters.

``fifty_visual.xml``  real STL appearance + simplified collision primitives
``fifty_train.xml``   collision primitives only (no meshes, no visual geoms)

Both files are generated from the same corrected robot model, so mass, COM,
inertia, joints, limits, actuators and the spawn keyframe cannot drift apart.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ..config import Config
from ..control.gas_spring import load_springs
from ..math3d import mat_to_quat_wxyz, rpy_to_mat
from .build_corrected import CorrectedAssets
from .kinematics import link_poses
from .urdf_io import Geometry, Joint, Link, RobotModel, Shape

def _fmt(value: float) -> str:
    """Shortest round-trip float formatting; MuJoCo parses full float64."""
    text = repr(float(value))
    return "0" if text == "-0.0" else text


def _vec(values) -> str:
    return " ".join(_fmt(v) for v in values)


def _fullinertia(inertia: np.ndarray) -> str:
    m = np.asarray(inertia, dtype=float)
    # MuJoCo order: ixx iyy izz ixy ixz iyz
    return " ".join(
        [_fmt(m[0, 0]), _fmt(m[1, 1]), _fmt(m[2, 2]), _fmt(m[0, 1]), _fmt(m[0, 2]), _fmt(m[1, 2])]
    )


def _geom_element(
    parent: ET.Element,
    shape: Shape,
    assets: dict[str, str],
    classes: dict[str, str],
    materials: dict[str, str] | None = None,
) -> ET.Element:
    geometry = shape.geometry
    attributes: dict[str, str] = {
        "pos": _vec(shape.xyz),
        "quat": _vec(mat_to_quat_wxyz(rpy_to_mat(shape.rpy))),
    }
    if geometry.kind == "mesh":
        attributes["type"] = "mesh"
        attributes["mesh"] = assets[geometry.mesh_file]
        attributes["class"] = classes["visual"]
        if materials and shape.rgba is not None:
            key = " ".join(_fmt(c) for c in shape.rgba)
            if key in materials:
                attributes["material"] = materials[key]
    elif geometry.kind == "box":
        attributes["type"] = "box"
        attributes["size"] = _vec(np.asarray(geometry.size) / 2.0)  # MuJoCo box size = half extents
        attributes["class"] = classes["collision"]
        if shape.rgba is not None:
            attributes["rgba"] = " ".join(_fmt(c) for c in shape.rgba)
    elif geometry.kind == "cylinder":
        attributes["type"] = "cylinder"
        attributes["size"] = f"{_fmt(geometry.size[0])} {_fmt(geometry.size[1] / 2.0)}"
        attributes["class"] = classes["collision"]
        if shape.rgba is not None:
            attributes["rgba"] = " ".join(_fmt(c) for c in shape.rgba)
    elif geometry.kind == "sphere":
        attributes["type"] = "sphere"
        attributes["size"] = _fmt(geometry.size[0])
        attributes["class"] = classes["collision"]
    else:
        raise ValueError(f"unsupported geometry kind {geometry.kind!r}")
    return ET.SubElement(parent, "geom", attributes)


def _mesh_asset_name(mesh_file: str) -> str:
    return Path(mesh_file).stem


def _children_map(robot: RobotModel) -> dict[str, list[Joint]]:
    return robot.child_map()


def _spawn_height(corrected: RobotModel, config: Config) -> tuple[float, dict[str, float]]:
    """Root height that puts the lowest wheel axis at ``radius + clearance``."""
    poses = link_poses(corrected, config.default_angles)
    wheel_axes = {
        name: float(poses[name].t[2]) for name in corrected.links if name.endswith("_wheel")
    }
    wheel_radius = config.wheel_radius
    clearance = float(config.spawn["spawn"]["wheel_clearance_m"])
    lowest = min(wheel_axes.values())
    root_z = (wheel_radius + clearance) - lowest
    return float(root_z), wheel_axes


def _write_body(
    parent: ET.Element,
    robot: RobotModel,
    link_name: str,
    children: dict[str, list[Joint]],
    config: Config,
    variant: str,
    mesh_assets: dict[str, str],
    classes: dict[str, str],
    spring_sites: dict[str, list[tuple[str, np.ndarray]]] | None = None,
    materials: dict[str, str] | None = None,
) -> None:
    link: Link = robot.links[link_name]
    attributes = {"name": link_name}
    joint: Joint | None = None
    if link_name != robot.root_link:
        joint = next(j for j in robot.joints.values() if j.child == link_name)
        attributes["pos"] = _vec(joint.xyz)
        attributes["quat"] = _vec(mat_to_quat_wxyz(rpy_to_mat(joint.rpy)))
    body = ET.SubElement(parent, "body", attributes)

    if link_name == robot.root_link:
        ET.SubElement(body, "freejoint", {"name": "root"})

    if link.inertial is not None:
        ET.SubElement(
            body,
            "inertial",
            {
                "pos": _vec(link.inertial.com),
                "mass": _fmt(link.inertial.mass),
                "fullinertia": _fullinertia(link.inertial.inertia),
            },
        )

    if joint is not None:
        joint_attributes: dict[str, str] = {
            "name": joint.name,
            "axis": _vec(joint.axis),
            "class": classes[config.joint_group(joint.name)],
        }
        if joint.lower is not None and joint.upper is not None:
            joint_attributes["range"] = f"{_fmt(joint.lower)} {_fmt(joint.upper)}"
        else:
            joint_attributes["limited"] = "false"
        ET.SubElement(body, "joint", joint_attributes)

    if variant == "visual":
        for shape in link.visuals:
            _geom_element(body, shape, mesh_assets, classes, materials)
    for shape in link.collisions:
        _geom_element(body, shape, mesh_assets, classes, materials)

    # Gas-spring anchor sites live on the body that actually carries them
    # (never both on the same body, never on worldbody).
    for site_name, site_pos in (spring_sites or {}).get(link_name, []):
        ET.SubElement(
            body,
            "site",
            {"name": site_name, "pos": _vec(site_pos), "size": "0.004", "rgba": "1 0.45 0.1 1"},
        )

    for child_joint in children.get(link_name, []):
        _write_body(parent=body, robot=robot, link_name=child_joint.child, children=children,
                    config=config, variant=variant, mesh_assets=mesh_assets, classes=classes,
                    spring_sites=spring_sites, materials=materials)


def _defaults(root: ET.Element, config: Config) -> dict[str, str]:
    defaults = ET.SubElement(root, "default")
    contact = config.robot["contact"]
    friction = _vec(contact["friction"])
    solref = _vec(contact["solref"])
    solimp = _vec(contact["solimp"])

    # Contact bitmasks.  The floor is contype=1/conaffinity=1.
    #   self_collision = False -> robot contype=2, conaffinity=1
    #       robot<->robot: (2&1)|(2&1) = 0   (disabled)
    #       robot<->floor: (2&1)|(1&1) = 1   (kept)
    # The first version disables robot self-collision on purpose: the
    # provisional base AABB box geometrically encloses the thighs and shins.
    self_collision = bool(contact["self_collision"])
    robot_contype = "1" if self_collision else "2"
    robot_conaffinity = "1"

    robot_default = ET.SubElement(defaults, "default", {"class": "robot"})
    ET.SubElement(
        robot_default,
        "geom",
        {
            "group": "3",
            "contype": robot_contype,
            "conaffinity": robot_conaffinity,
            "friction": friction,
            "solref": solref,
            "solimp": solimp,
        },
    )

    visual = ET.SubElement(defaults, "default", {"class": "visual"})
    ET.SubElement(
        visual,
        "geom",
        {"group": "2", "contype": "0", "conaffinity": "0", "mass": "0", "density": "0"},
    )

    collision = ET.SubElement(defaults, "default", {"class": "collision"})
    ET.SubElement(
        collision,
        "geom",
        {
            "group": "3",
            "contype": robot_contype,
            "conaffinity": robot_conaffinity,
            "friction": friction,
            "solref": solref,
            "solimp": solimp,
        },
    )

    for group in ("hip", "knee", "wheel"):
        element = ET.SubElement(defaults, "default", {"class": group})
        ET.SubElement(
            element,
            "joint",
            {
                "damping": _fmt(config.robot["joints"]["damping"][group]),
                "frictionloss": _fmt(config.robot["joints"]["frictionloss"][group]),
                "armature": _fmt(config.robot["joints"]["armature"][group]),
            },
        )
    return {"visual": "visual", "collision": "collision", "hip": "hip", "knee": "knee", "wheel": "wheel"}


def emit_mjcf(
    config: Config,
    corrected: RobotModel,
    variant: str,
    spawn_z: float,
) -> str:
    model_name = str(config.robot["robot"]["mjcf_model_name"])
    root = ET.Element("mujoco", {"model": f"{model_name}_{variant}"})

    meshdir = str(config.robot["geometry"]["mesh_subdir"])
    ET.SubElement(
        root,
        "compiler",
        {
            "angle": "radian",
            "meshdir": f"../corrected/{meshdir}",
            "autolimits": "true",
            # Explicit inertias are never silently repaired: an invalid tensor
            # must fail `validate-mjcf`, not be balanced away by the compiler.
            "balanceinertia": "false",
        },
    )
    simulation = config.robot["simulation"]
    ET.SubElement(
        root,
        "option",
        {
            "timestep": _fmt(simulation["timestep_s"]),
            "integrator": str(simulation["integrator"]),
            "cone": str(simulation["cone"]),
            "gravity": _vec(simulation["gravity"]),
        },
    )
    # Larger offscreen framebuffer so `render` can produce inspection images
    # above the 640x480 default.
    visual_element = ET.SubElement(root, "visual")
    ET.SubElement(
        visual_element,
        "global",
        {"offwidth": "1920", "offheight": "1080", "fovy": "45"},
    )

    classes = _defaults(root, config)

    mesh_assets: dict[str, str] = {}
    materials: dict[str, str] = {}
    if variant == "visual":
        asset = ET.SubElement(root, "asset")
        for link in corrected.links.values():
            for shape in link.visuals:
                if shape.geometry.kind != "mesh":
                    continue
                mesh_file = str(shape.geometry.mesh_file)
                name = _mesh_asset_name(mesh_file)
                if name not in mesh_assets:
                    # MuJoCo's STL decoder caps at 200k faces, so the visual model
                    # uses OBJ derivatives generated next to the untouched STLs.
                    mesh_assets[mesh_file] = name
                    ET.SubElement(
                        asset, "mesh", {"name": name, "file": Path(mesh_file).with_suffix(".obj").name}
                    )
                if shape.rgba is not None:
                    key = " ".join(_fmt(c) for c in shape.rgba)
                    if key not in materials:
                        material_name = f"mat_{len(materials)}"
                        materials[key] = material_name
                        ET.SubElement(
                            asset, "material", {"name": material_name, "rgba": key, "specular": "0.2", "shininess": "0.3"}
                        )

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "size": "10 10 0.1",
            "pos": "0 0 0",
            "contype": "1",
            "conaffinity": "1",
            "friction": _vec(config.robot["contact"]["ground_friction"]),
            "group": "1",
            "rgba": "0.35 0.36 0.38 1",
        },
    )
    # Fixed lights: the default headlight alone gives a flat, unreadable image
    # in the fixed-camera snapshots.
    ET.SubElement(
        worldbody,
        "light",
        {"name": "key", "pos": "1.4 -1.6 2.2", "dir": "-0.5 0.6 -1", "diffuse": "0.8 0.8 0.8", "specular": "0.2 0.2 0.2"},
    )
    ET.SubElement(
        worldbody,
        "light",
        {"name": "fill", "pos": "-1.6 1.2 1.4", "dir": "0.6 -0.5 -1", "diffuse": "0.35 0.35 0.4", "specular": "0.05 0.05 0.05"},
    )

    children = _children_map(corrected)

    spring_sites: dict[str, list[tuple[str, np.ndarray]]] = {}
    springs = load_springs(config) if config.gas_spring_configured else []
    for spring in springs:
        spring_sites.setdefault(spring.body_a, []).append((f"{spring.name}_a", spring.xyz_a))
        spring_sites.setdefault(spring.body_b, []).append((f"{spring.name}_b", spring.xyz_b))
        if spring.body_a == spring.body_b:
            raise ValueError(f"{spring.name}: both anchors are on {spring.body_a}; that cannot support anything")

    _write_body(
        parent=worldbody,
        robot=corrected,
        link_name=corrected.root_link,
        children=children,
        config=config,
        variant=variant,
        mesh_assets=mesh_assets,
        classes=classes,
        spring_sites=spring_sites,
        materials=materials,
    )

    if springs:
        tendon = ET.SubElement(root, "tendon")
        for spring in springs:
            spatial = ET.SubElement(
                tendon,
                "spatial",
                {"name": f"{spring.name}_length", "width": "0.001"},
            )
            ET.SubElement(spatial, "site", {"site": f"{spring.name}_a"})
            ET.SubElement(spatial, "site", {"site": f"{spring.name}_b"})

    actuator = ET.SubElement(root, "actuator")
    for joint_name in config.joint_order:
        limit = _fmt(config.effort_limit(joint_name))
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": f"{joint_name}_motor",
                "joint": joint_name,
                "ctrllimited": "true",
                "forcelimited": "true",
                "ctrlrange": f"-{limit} {limit}",
                "forcerange": f"-{limit} {limit}",
            },
        )
    for spring in springs:
        # Range is symmetric: positive ctrl is tendon tension (shortens the
        # spring), negative ctrl is the strut pushing the anchors apart.
        limit = _fmt(spring.force_limit)
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": f"{spring.name}_force",
                "tendon": f"{spring.name}_length",
                "gear": "1",
                "ctrllimited": "true",
                "ctrlrange": f"-{limit} {limit}",
                "forcelimited": "true",
                "forcerange": f"-{limit} {limit}",
            },
        )

    quat = np.array(config.spawn["spawn"]["root_quat_wxyz"], dtype=float)
    quat = quat / np.linalg.norm(quat)
    xy = config.spawn["spawn"]["root_xy_m"]
    qpos_values = [float(xy[0]), float(xy[1]), float(spawn_z)] + quat.tolist() + [
        float(config.default_angles[name]) for name in config.joint_order
    ]
    keyframe = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframe, "key", {"name": "spawn", "qpos": _vec(qpos_values)})

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    return ET.tostring(root, encoding="unicode")


@dataclass
class MjcfPaths:
    visual: Path
    train: Path
    spawn_z: float
    wheel_axes_z: dict[str, float]


def write_mjcf_variants(config: Config, assets: CorrectedAssets) -> MjcfPaths:
    out_dir = Path(__file__).resolve().parents[3] / "assets" / "mjcf"
    out_dir.mkdir(parents=True, exist_ok=True)

    spawn_z, wheel_axes = _spawn_height(assets.robot, config)
    visual = out_dir / "fifty_visual.xml"
    train = out_dir / "fifty_train.xml"
    visual.write_text(emit_mjcf(config, assets.robot, "visual", spawn_z) + "\n", encoding="utf-8")
    train.write_text(emit_mjcf(config, assets.robot, "train", spawn_z) + "\n", encoding="utf-8")
    return MjcfPaths(visual=visual, train=train, spawn_z=spawn_z, wheel_axes_z=wheel_axes)

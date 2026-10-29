"""Collision primitive generation from the corrected meshes.

Rules (doc section 3.4):
    base_link    box from the full mesh AABB (provisional, coverage re-checked)
    thigh/shin   cylinder along the link x axis, from a 2-98% trimmed bbox
    wheels       cylinder whose axis is the wheel joint axis and whose centre is
                 exactly the link origin; radius from robot.yaml (provisional)

Visual meshes never take part in contacts and never contribute mass/inertia
(the bodies carry explicit `<inertial>` data).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import Config
from . import stl
from .urdf_io import Geometry, Shape


@dataclass(frozen=True)
class CollisionSpec:
    link: str
    kind: str
    detail: dict
    vertices_used: int


def _link_kind(link_name: str) -> str:
    if link_name == "base_link":
        return "base_box"
    if link_name.endswith("_wheel"):
        return "wheel_cylinder"
    return "leg_cylinder"


def build_collisions(
    link_names: list[str],
    mesh_transforms: dict[str, object],
    mesh_paths: dict[str, str],
    config: Config,
) -> tuple[dict[str, list[Shape]], dict[str, CollisionSpec]]:
    """Return ``{link: [Shape, ...]}`` plus a human-readable spec per link."""
    geometry_cfg = config.robot["geometry"]
    trim_low = float(geometry_cfg["mesh_trim_lower"])
    trim_high = float(geometry_cfg["mesh_trim_upper"])
    radius_fraction = float(geometry_cfg["leg_cylinder_radius_fraction"])
    radius_min = float(geometry_cfg["leg_cylinder_radius_min_m"])
    radius_max = float(geometry_cfg["leg_cylinder_radius_max_m"])
    wheel_radius = float(geometry_cfg["wheel_radius_m"])
    base_scale = float(geometry_cfg["base_collision_scale"])

    shapes: dict[str, list[Shape]] = {}
    specs: dict[str, CollisionSpec] = {}

    for link_name in link_names:
        transform = mesh_transforms[link_name]
        vertices = stl.stl_vertices(mesh_paths[link_name], transform)
        kind = _link_kind(link_name)

        if kind == "base_box":
            low, high = stl.aabb(vertices)
            size = (high - low) * base_scale
            centre = 0.5 * (low + high)
            shapes[link_name] = [
                Shape(Geometry("box", size=size), centre, np.zeros(3), (0.85, 0.35, 0.25, 0.35))
            ]
            specs[link_name] = CollisionSpec(
                link_name,
                "box",
                {
                    "size_m": size.tolist(),
                    "aabb_min_m": low.tolist(),
                    "aabb_max_m": high.tolist(),
                    "scale": base_scale,
                },
                int(vertices.shape[0]),
            )

        elif kind == "leg_cylinder":
            low, high = stl.trimmed_aabb(vertices, trim_low, trim_high)
            extent = high - low
            radius = float(np.clip(radius_fraction * max(extent[1], extent[2]), radius_min, radius_max))
            length = float(extent[0])
            centre = 0.5 * (low + high)
            shapes[link_name] = [
                Shape(
                    Geometry("cylinder", size=np.array([radius, length])),
                    centre,
                    np.array([0.0, np.pi / 2.0, 0.0]),
                    (0.85, 0.35, 0.25, 0.35),
                )
            ]
            specs[link_name] = CollisionSpec(
                link_name,
                "cylinder_x",
                {
                    "radius_m": radius,
                    "length_m": length,
                    "centre_m": centre.tolist(),
                    "trimmed_extent_m": extent.tolist(),
                    "trim": [trim_low, trim_high],
                },
                int(vertices.shape[0]),
            )

        else:  # wheel
            length = stl.extent_along(vertices, 2, trim_low, trim_high)
            shapes[link_name] = [
                Shape(
                    Geometry("cylinder", size=np.array([wheel_radius, float(length)])),
                    np.zeros(3),
                    np.zeros(3),
                    (0.85, 0.35, 0.25, 0.35),
                )
            ]
            specs[link_name] = CollisionSpec(
                link_name,
                "cylinder_z",
                {
                    "radius_m": wheel_radius,
                    "length_m": float(length),
                    "centre_m": [0.0, 0.0, 0.0],
                    "radius_status": geometry_cfg["wheel_radius_status"],
                    "mesh_radius_p95_m": stl.radius_about_axis(vertices, 2),
                },
                int(vertices.shape[0]),
            )

    return shapes, specs

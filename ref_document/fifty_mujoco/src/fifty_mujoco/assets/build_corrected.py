"""P1: rebuild visual / centre-of-mass / inertia with a single rigid transform.

For every link the transform is computed once, from two complete joint trees
evaluated at the same physical assembly pose::

    T_new<-old = inv(T_new(q_default)) @ T_base_from_source @ T_old(q=0)

The visual mesh origin, the centre of mass and the inertia tensor are moved by
that transform.  The inertia tensor is rotated exactly once (``R_ni``), and the
written ``inertial/origin/rpy`` is zero, so a downstream loader cannot rotate it
a second time.
"""

from __future__ import annotations

import json
import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ..config import Config
from ..math3d import (
    Transform,
    inertia_is_physically_valid,
    principal_inertia,
    rpy_to_mat,
)
from . import colliders, mesh_export, stl
from .kinematics import link_poses
from .urdf_io import Inertial, RobotModel, Shape, load_urdf, save_urdf


@dataclass
class CorrectedAssets:
    robot: RobotModel
    mesh_transforms: dict[str, Transform]
    report: dict[str, Any]


def _link_mesh_path(config: Config, robot: RobotModel, link_name: str) -> Path:
    shapes = robot.links[link_name].visuals
    if not shapes or shapes[0].geometry.kind != "mesh":
        raise ValueError(f"{link_name}: expected a mesh visual from the source package")
    mesh_file = str(shapes[0].geometry.mesh_file)
    package_dir = config.workspace_path(config.robot["robot"]["source_package"])
    path = package_dir / mesh_file
    if not path.is_file():  # some exports keep only the basename
        path = config.source_mesh_dir / Path(mesh_file).name
    if not path.is_file():
        raise FileNotFoundError(f"{link_name}: mesh not found ({mesh_file})")
    return path


def _link_meshes(config: Config, source: RobotModel) -> dict[str, tuple[str, Path]]:
    result: dict[str, tuple[str, Path]] = {}
    for link_name in source.links:
        path = _link_mesh_path(config, source, link_name)
        result[link_name] = (path.name, path)
    return result


def _hardlink(src: Path, dst: Path) -> str:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        dst.unlink()
    try:
        os.link(src, dst)
        return "hardlink"
    except OSError:
        shutil.copy2(src, dst)
        return "copy"


def build_corrected_assets(config: Config) -> CorrectedAssets:
    source = load_urdf(config.source_urdf)
    training = load_urdf(config.training_urdf)

    base_transform = config.base_transform
    base_transform.assert_valid("source->training base transform")

    poses_source = link_poses(source, {})
    poses_training = link_poses(training, config.default_angles)

    corrected = training.copy()
    corrected.name = f"{config.robot['robot']['mjcf_model_name']}_corrected"
    corrected.root_link = training.root_link

    mesh_files = _link_meshes(config, source)
    mesh_transforms: dict[str, Transform] = {}
    report_links: dict[str, Any] = {}

    for link_name in training.links:
        transform = poses_training[link_name].inverse() @ base_transform @ poses_source[link_name]
        correction = config.mesh_correction(link_name)
        if correction is not None:
            # Applied in the link frame; for the root link that equals the
            # training base frame, so the stated rotation is directly visible.
            transform = correction @ transform
        transform.assert_valid(f"{link_name}: link transform")
        mesh_transforms[link_name] = transform

        source_link = source.links[link_name]
        target_link = corrected.links[link_name]

        # ---------------------------------------------------------- visuals
        visuals: list[Shape] = []
        visual_origins = []
        for shape in source_link.visuals:
            moved = transform @ shape.transform()
            xyz, rpy = moved.to_xyz_rpy()
            visuals.append(Shape(shape.geometry.copy(), xyz, rpy, shape.rgba))
            visual_origins.append(
                {
                    "source_xyz": shape.xyz.tolist(),
                    "source_rpy": shape.rpy.tolist(),
                    "corrected_xyz": xyz.tolist(),
                    "corrected_rpy": rpy.tolist(),
                }
            )
        if not visuals:
            raise ValueError(f"{link_name}: source link has no visual geometry")
        target_link.visuals = visuals

        # --------------------------------------------------------- inertials
        if source_link.inertial is None:
            target_link.inertial = None
            inertial_report = {"mass_kg": 0.0}
        else:
            source_inertial = source_link.inertial
            rotation_inertial = rpy_to_mat(source_inertial.rpy)
            inertia_link_old = rotation_inertial @ source_inertial.inertia @ rotation_inertial.T
            inertia_new = transform.R @ inertia_link_old @ transform.R.T
            com_new = transform.apply(source_inertial.com)
            target_link.inertial = Inertial(
                mass=float(source_inertial.mass),
                com=com_new,
                rpy=np.zeros(3),
                inertia=inertia_new,
            )
            valid, reason = inertia_is_physically_valid(inertia_new)
            inertial_report = {
                "mass_kg": float(source_inertial.mass),
                "source_com_m": source_inertial.com.tolist(),
                "corrected_com_m": com_new.tolist(),
                "source_inertial_rpy": source_inertial.rpy.tolist(),
                "corrected_inertial_rpy": [0.0, 0.0, 0.0],
                "source_inertia_kgm2": source_inertial.inertia.tolist(),
                "corrected_inertia_kgm2": inertia_new.tolist(),
                "principal_moments_kgm2": principal_inertia(inertia_new).tolist(),
                "physically_valid": bool(valid),
                "validity_note": reason,
            }
            if not valid:
                raise ValueError(f"{link_name}: corrected inertia invalid ({reason})")

        report_links[link_name] = {
            "transform_R": transform.R.tolist(),
            "transform_t_m": transform.t.tolist(),
            "mesh_correction_applied": correction is not None,
            "visual": visual_origins,
            "inertial": inertial_report,
            "mesh_file": mesh_files[link_name][0],
        }

    # ------------------------------------------------------------- collisions
    link_names = [name for name in corrected.links]
    collision_shapes, collision_specs = colliders.build_collisions(
        link_names,
        mesh_transforms,
        {name: str(mesh_files[name][1]) for name in link_names},
        config,
    )
    for link_name, shapes in collision_shapes.items():
        corrected.links[link_name].collisions = shapes

    # -------------------------------------------------------- write artifacts
    corrected_dir = config.robot.get("_corrected_dir")
    corrected_dir = Path(corrected_dir) if corrected_dir else (
        Path(__file__).resolve().parents[3] / "assets" / "corrected"
    )
    mesh_dir = corrected_dir / str(config.robot["geometry"]["mesh_subdir"])
    link_modes: dict[str, str] = {}
    obj_exports: dict[str, dict] = {}
    for name, (filename, path) in mesh_files.items():
        link_modes[name] = _hardlink(path, mesh_dir / filename)
        obj_exports[name] = mesh_export.ensure_obj(path, mesh_dir)

    mesh_prefix = f"{config.robot['geometry']['mesh_subdir']}"
    for link_name, link in corrected.links.items():
        for shape in link.visuals:
            shape.geometry.mesh_file = f"{mesh_prefix}/{mesh_files[link_name][0]}"

    urdf_path = save_urdf(corrected, corrected_dir / "fifty_corrected.urdf")

    # ------------------------------------------------------- geometry report
    base_vertices = stl.stl_vertices(mesh_files["base_link"][1], mesh_transforms["base_link"])
    base_low, base_high = stl.aabb(base_vertices)
    top_point = stl.top_point(base_vertices)

    report: dict[str, Any] = {
        "stage": "P1",
        "source_urdf": str(config.source_urdf),
        "source_urdf_sha256": stl.sha256(config.source_urdf),
        "training_urdf": str(config.training_urdf),
        "corrected_urdf": str(urdf_path),
        "corrected_urdf_sha256": stl.sha256(urdf_path),
        "base_transform": base_transform.as_dict(),
        "default_joint_angles_rad": config.default_angles,
        "links": report_links,
        "collisions": {
            name: {"kind": spec.kind, "detail": spec.detail, "vertices": spec.vertices_used}
            for name, spec in collision_specs.items()
        },
        "mesh_links": link_modes,
        "mesh_exports": obj_exports,
        "mesh_export_note": (
            "OBJ derivatives are used by the visual MJCF because MuJoCo's STL decoder "
            "rejects files above 200k faces; the source STL files are untouched"
        ),
        "base_geometry": {
            "aabb_min_m": base_low.tolist(),
            "aabb_max_m": base_high.tolist(),
            "top_point_local_m": top_point.tolist(),
            "top_point_source": "highest CAD vertex of base_link.STL in the corrected base frame",
        },
        "mass": {
            "source_kg": source.total_mass(),
            "corrected_kg": corrected.total_mass(),
            "delta_kg": corrected.total_mass() - source.total_mass(),
        },
    }
    report["checks"] = build_corrected_checks(config, source, training, corrected, mesh_transforms, report)
    report["status"] = "passed" if all(check["passed"] for check in report["checks"].values()) else "failed"

    _write_json(report, corrected_dir / "transform_report.json")
    return CorrectedAssets(robot=corrected, mesh_transforms=mesh_transforms, report=report)


def build_corrected_checks(
    config: Config,
    source: RobotModel,
    training: RobotModel,
    corrected: RobotModel,
    mesh_transforms: dict[str, Transform],
    report: dict[str, Any],
) -> dict[str, Any]:
    checks: dict[str, Any] = {}

    mass_delta = abs(report["mass"]["delta_kg"])
    checks["mass_preserved"] = {
        "passed": mass_delta <= 1e-9,
        "delta_kg": mass_delta,
    }

    orthonormal = all(
        np.allclose(t.R.T @ t.R, np.eye(3), atol=1e-10) and abs(np.linalg.det(t.R) - 1.0) < 1e-9
        for t in mesh_transforms.values()
    )
    checks["all_transforms_proper_rotations"] = {"passed": bool(orthonormal)}

    # The transform must be applied exactly once.  For links whose transform is
    # (nearly) identity a double application is indistinguishable, so only the
    # links where it *is* measurable are used as evidence.
    detectable: dict[str, float] = {}
    double_applied: list[str] = []
    for link_name, transform in mesh_transforms.items():
        once = transform.t
        twice = (transform @ transform).t
        difference = float(np.linalg.norm(twice - once))
        if difference > 1e-6:
            detectable[link_name] = difference
            stored = corrected.links[link_name].visuals[0].xyz
            if np.allclose(stored, twice, atol=1e-9) and not np.allclose(stored, once, atol=1e-9):
                double_applied.append(link_name)
    checks["transform_not_applied_twice"] = {
        "passed": not double_applied,
        "offending": double_applied,
        "links_where_double_application_is_detectable": detectable,
        "note": "links with an identity transform cannot provide evidence either way",
    }

    identities = all(
        float(np.abs(link.inertial.rpy).max()) < 1e-12
        for link in corrected.links.values()
        if link.inertial is not None
    )
    checks["inertial_origin_rpy_is_zero"] = {"passed": bool(identities)}

    inertia_ok = True
    inertia_details = {}
    for link_name, link in corrected.links.items():
        if link.inertial is None:
            continue
        valid, reason = inertia_is_physically_valid(link.inertial.inertia)
        inertia_details[link_name] = {"valid": valid, "note": reason}
        inertia_ok = inertia_ok and valid
    checks["inertia_physically_valid"] = {"passed": inertia_ok, "per_link": inertia_details}

    inertial_mass_delta = {}
    for link_name, link in corrected.links.items():
        if link.inertial is None:
            continue
        inertial_mass_delta[link_name] = abs(link.inertial.mass - source.links[link_name].inertial.mass)
    checks["per_link_mass_preserved"] = {
        "passed": all(value <= 1e-12 for value in inertial_mass_delta.values()),
        "max_delta_kg": max(inertial_mass_delta.values(), default=0.0),
    }

    # Collision coverage: every collision shape must be finite and non-degenerate.
    degenerate = []
    for link_name, link in corrected.links.items():
        for shape in link.collisions:
            if shape.geometry.size is None or not np.all(np.isfinite(shape.geometry.size)):
                degenerate.append(link_name)
                continue
            if np.any(shape.geometry.size <= 0.0):
                degenerate.append(link_name)
    checks["collision_shapes_non_degenerate"] = {"passed": not degenerate, "offending": degenerate}

    # The corrected visual origin must be reproducible from the reported
    # transforms (guards against hand-edited numbers).
    visual_mismatch = []
    for link_name, transform in mesh_transforms.items():
        visual = corrected.links[link_name].visuals[0]
        xyz, _ = transform.to_xyz_rpy()
        if not np.allclose(visual.xyz, xyz, atol=1e-9):
            visual_mismatch.append(link_name)
    checks["visual_origins_match_transforms"] = {
        "passed": not visual_mismatch,
        "offending": visual_mismatch,
    }

    # Joint frames are inherited from the validated G1 chain and must not move.
    joint_mismatch = [
        name
        for name in training.joints
        if not np.allclose(corrected.joints[name].xyz, training.joints[name].xyz, atol=1e-12)
        or not np.allclose(corrected.joints[name].rpy, training.joints[name].rpy, atol=1e-12)
        or not np.allclose(corrected.joints[name].axis, training.joints[name].axis, atol=1e-12)
    ]
    checks["joint_frames_inherited_from_validated_chain"] = {
        "passed": not joint_mismatch,
        "offending": joint_mismatch,
    }
    return checks


def _write_json(payload: dict[str, Any], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path

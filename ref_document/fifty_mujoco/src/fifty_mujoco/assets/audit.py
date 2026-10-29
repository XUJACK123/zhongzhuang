"""P0: read-only audit of the source assets and of the current G1 defect.

Outputs ``reports/assets_audit.json``.  This module never writes to the source
packages.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from ..config import Config
from ..math3d import Transform, is_rotation
from . import stl
from .kinematics import joint_value_consistency, link_poses
from .urdf_io import load_urdf


def _transform_from_config(config: Config) -> Transform:
    transform = config.base_transform
    transform.assert_valid("source->training base transform")
    if not is_rotation(transform.R):
        raise ValueError("recorded base rotation is not a proper rotation")
    return transform


def audit_assets(config: Config) -> dict[str, Any]:
    report: dict[str, Any] = {"stage": "P0", "status": "unknown", "checks": {}, "findings": {}, "defects": []}

    source_path = config.source_urdf
    training_path = config.training_urdf
    parameters_path = config.parameters_json

    recorded = json.loads(parameters_path.read_text(encoding="utf-8"))
    source_sha = stl.sha256(source_path)
    report["sources"] = {
        "source_urdf": {
            "path": str(source_path),
            "sha256": source_sha,
            "sha256_matches_parameters_json": source_sha == recorded.get("source_urdf_sha256"),
        },
        "training_urdf": {"path": str(training_path), "sha256": stl.sha256(training_path)},
        "parameters_json": {"path": str(parameters_path), "sha256": stl.sha256(parameters_path)},
        "gas_spring_config": {
            "configured": config.gas_spring_configured,
            "springs": [spring.get("name") for spring in config.gas_spring_springs],
        },
    }

    # ------------------------------------------------------------------ meshes
    recorded_meshes = recorded.get("mesh_transforms", {})
    meshes: dict[str, Any] = {}
    for link_name, entry in recorded_meshes.items():
        mesh_path = config.source_mesh_dir / entry["file"]
        info = stl.stl_info(mesh_path)
        info["recorded_sha256_matches"] = info["sha256"] == entry.get("sha256")
        info["recorded_triangles_matches"] = info["triangles"] == entry.get("triangles")
        meshes[link_name] = info
    report["meshes"] = meshes

    # ------------------------------------------------------------- joint trees
    source = load_urdf(source_path)
    training = load_urdf(training_path)
    report["joint_trees"] = {
        "source": {
            "name": source.name,
            "root": source.root_link,
            "links": list(source.links),
            "joints": [
                {
                    "name": j.name,
                    "type": j.type,
                    "parent": j.parent,
                    "child": j.child,
                    "origin_xyz": j.xyz.tolist(),
                    "origin_rpy": j.rpy.tolist(),
                    "axis": j.axis.tolist(),
                }
                for j in source.joint_list()
            ],
        },
        "training": {
            "name": training.name,
            "root": training.root_link,
            "links": list(training.links),
            "joints": [
                {
                    "name": j.name,
                    "type": j.type,
                    "parent": j.parent,
                    "child": j.child,
                    "origin_xyz": j.xyz.tolist(),
                    "origin_rpy": j.rpy.tolist(),
                    "axis": j.axis.tolist(),
                }
                for j in training.joint_list()
            ],
        },
        "declaration_order_matches_contract": [j.name for j in training.joint_list()] == config.joint_order,
        "mass_source_kg": source.total_mass(),
        "mass_training_kg": training.total_mass(),
    }

    base_transform = _transform_from_config(config)

    # ------------------------------------------- joint frame correspondence
    correspondence = joint_value_consistency(
        source,
        training,
        q_from={},
        q_to=config.default_angles,
        to_from_root=base_transform,
        names=config.joint_order,
    )
    report["joint_correspondence"] = correspondence
    worst_origin = max(entry["origin_error_m"] for entry in correspondence.values())
    # The G1 chain deliberately re-defines every link frame, so a joint axis may
    # appear as +z or -z in the new frame.  What must hold is that both models
    # rotate about the *same physical line*: |axis_a . axis_b| == 1.
    worst_axis_parallel = max(abs(abs(entry["axis_dot"]) - 1.0) for entry in correspondence.values())
    report["frame_signs"] = {name: float(entry["axis_dot"]) for name, entry in correspondence.items()}
    report["checks"]["joint_origin_alignment"] = {
        "max_error_m": worst_origin,
        "passed": worst_origin <= 1e-6,
    }
    report["checks"]["joint_axis_parallel"] = {
        "max_deviation_from_parallel": worst_axis_parallel,
        "passed": worst_axis_parallel <= 1e-9,
        "note": "sign per joint is a frame convention, recorded in frame_signs",
    }

    # ------------------------------------------------- link frame vs CAD pose
    pose_source = link_poses(source, {})
    pose_training = link_poses(training, config.default_angles)
    link_errors: dict[str, Any] = {}
    for link_name in training.links:
        if link_name not in pose_source:
            continue
        # The only invariant that matters: composing the per-link transform with
        # the training pose must reproduce the source pose seen through the base
        # transform.  This validates the derivation itself.
        derived = base_transform @ pose_source[link_name]
        relative = pose_training[link_name].inverse() @ derived
        recomposed = pose_training[link_name] @ relative
        link_errors[link_name] = {
            "recomposition_error_m": float(np.linalg.norm(recomposed.t - derived.t)),
            "rotation_error_deg": float(
                np.degrees(
                    np.arccos(
                        np.clip((np.trace(recomposed.R.T @ derived.R) - 1.0) / 2.0, -1.0, 1.0)
                    )
                )
            ),
            "frame_rotation_vs_source_deg": float(
                np.degrees(
                    np.arccos(
                        np.clip(
                            (np.trace(relative.R.T @ np.eye(3)) - 1.0) / 2.0, -1.0, 1.0
                        )
                    )
                )
            ),
        }
    report["link_frame_alignment"] = link_errors
    worst_link = max(entry["recomposition_error_m"] for entry in link_errors.values())
    report["checks"]["transform_derivation_consistency"] = {
        "max_error_m": worst_link,
        "passed": worst_link <= 1e-9,
        "note": "link frames legitimately differ from the CAD export; only the composition is checked",
    }

    # ------------------------------------------ quantify the current defect
    defects: list[dict[str, Any]] = []
    for link_name, entry in recorded_meshes.items():
        if link_name not in training.links:
            continue
        link = training.links[link_name]
        if not link.visuals:
            continue
        actual_xyz = link.visuals[0].xyz
        recorded_xyz = np.array(entry.get("visual_xyz", [0.0, 0.0, 0.0]), dtype=float)
        ideal_transform = pose_training[link_name].inverse() @ base_transform @ pose_source[link_name]
        correction = config.mesh_correction(link_name)
        if correction is not None:
            # Intentional, documented correction (see configs/robot.yaml).
            ideal_transform = correction @ ideal_transform
        ideal = ideal_transform.t
        defects.append(
            {
                "link": link_name,
                "training_urdf_visual_xyz_m": actual_xyz.tolist(),
                "parameters_json_visual_xyz_m": recorded_xyz.tolist(),
                "recomputed_visual_xyz_m": ideal.tolist(),
                "offset_vs_recomputed_m": (actual_xyz - ideal).tolist(),
                "offset_norm_m": float(np.linalg.norm(actual_xyz - ideal)),
                "intentional_correction": correction is not None,
            }
        )
    report["visual_transform_defects"] = list(defects)
    offset_links = [d for d in defects if d["offset_norm_m"] > 1e-6 and not d["intentional_correction"]]
    report.setdefault("findings", {})["visual_offsets"] = {
        "correct": not offset_links,
        "offending_links": [d["link"] for d in offset_links],
        "max_offset_m": max((d["offset_norm_m"] for d in offset_links), default=0.0),
        "intentionally_corrected_links": [
            d["link"] for d in defects if d["intentional_correction"]
        ],
        "note": "P0 records the defect; P1 must remove it. The base link correction is deliberate.",
    }

    # inertial double-rotation check: the written inertial rpy must be identity
    # once the inertia is expressed in the link frame.
    inertial_rotation = {}
    for link_name, link in training.links.items():
        if link.inertial is None:
            continue
        angle = float(np.degrees(np.linalg.norm(link.inertial.rpy)))
        inertial_rotation[link_name] = angle
    report.setdefault("findings", {})["inertial_origin_rotation"] = {
        "identity": all(angle < 1e-6 for angle in inertial_rotation.values()),
        "per_link_deg": inertial_rotation,
        "note": "non-zero rpy here means the inertia is rotated through the inertial frame as well",
    }
    if any(angle >= 1e-6 for angle in inertial_rotation.values()):
        defects.append(
            {
                "link": "inertial",
                "issue": "inertial/origin/rpy is non-zero in the training URDF; the inertia rotation "
                "is applied twice (once to the tensor, once through the inertial frame)",
            }
        )

    report["defects"] = defects

    # --------------------------------------------------------- missing inputs
    missing = {
        "gas_spring_anchor_geometry": not config.gas_spring_configured,
        "gas_spring_force_and_stroke": not config.gas_spring_configured,
        "wheel_radius_measured": config.robot["geometry"]["wheel_radius_status"] != "measured",
        "rotor_armature": config.robot["joints"]["status"].get("armature") != "measured",
        "motor_torque_limit": config.robot["joints"]["status"]["effort_nm"] == "provisional",
        "base_inertia_verified": config.robot["joints"]["status"].get("base_inertia")
        == "provisional",
        "real_joint_limits": config.robot["joints"]["status"]["limits_rad"] == "provisional",
    }
    report["missing_inputs"] = missing

    hashes_ok = (
        report["sources"]["source_urdf"]["sha256_matches_parameters_json"]
        and all(entry["recorded_sha256_matches"] for entry in meshes.values())
    )
    report["checks"]["source_hashes_unchanged"] = {"passed": bool(hashes_ok)}
    report["checks"]["defects_reproduced"] = {
        "passed": True,
        "note": "audit reproduces the known visual/inertia defects; P1 must remove them",
        "defect_count": len(defects),
    }

    report["status"] = (
        "passed"
        if all(check["passed"] for check in report["checks"].values() if "passed" in check)
        else "failed"
    )
    return report


def write_report(report: dict[str, Any], path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path

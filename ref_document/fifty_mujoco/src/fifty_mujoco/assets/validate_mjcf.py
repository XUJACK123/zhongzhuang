"""Release checks for the generated MJCF (doc section 3.6).

Every check either passes with a recorded number or fails with an explicit
reason.  "The XML loads" and "the total mass matches" are not sufficient.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..config import Config
from ..math3d import inertia_is_physically_valid, principal_inertia
from .build_corrected import CorrectedAssets
from .build_mjcf import MjcfPaths
from .kinematics import link_poses
from .urdf_io import RobotModel


def _body_id(model: mujoco.MjModel, name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise ValueError(f"body {name!r} not found in model")
    return body_id


def _joint_id(model: mujoco.MjModel, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError(f"joint {name!r} not found in model")
    return joint_id


def _set_qpos(model: mujoco.MjModel, data: mujoco.MjData, q: dict[str, float], root_z: float = 0.0) -> None:
    data.qpos[:] = 0.0
    data.qpos[0:3] = (0.0, 0.0, root_z)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    for joint_name, value in q.items():
        joint_id = _joint_id(model, joint_name)
        address = int(model.jnt_qposadr[joint_id])
        data.qpos[address] = value


def fk_parity_check(
    model: mujoco.MjModel,
    corrected: RobotModel,
    config: Config,
    samples: int = 200,
    seed: int = 20261028,
) -> dict[str, Any]:
    data = mujoco.MjData(model)
    rng = np.random.default_rng(seed)
    limits = config.joint_limits
    worst = {"position_m": 0.0, "rotation_deg": 0.0}
    worst_case = None

    body_ids = {name: _body_id(model, name) for name in corrected.links}

    for index in range(samples):
        if index == 0:
            q = dict(config.default_angles)
        else:
            q = {}
            for name in config.joint_order:
                limit = limits[name]
                if limit is None:
                    q[name] = float(rng.uniform(-np.pi, np.pi))
                else:
                    low, high = limit
                    margin = 0.05 * (high - low)
                    q[name] = float(rng.uniform(low + margin, high - margin))
        _set_qpos(model, data, q)
        mujoco.mj_kinematics(model, data)

        poses = link_poses(corrected, q)
        for name, body_id in body_ids.items():
            reference = poses[name]
            position_error = float(np.linalg.norm(data.xpos[body_id] - reference.t))
            rotation = data.xmat[body_id].reshape(3, 3)
            cosine = (np.trace(rotation.T @ reference.R) - 1.0) / 2.0
            angle = float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))
            if position_error > worst["position_m"]:
                worst["position_m"] = position_error
                worst_case = {"sample": index, "link": name, "q": q}
            worst["rotation_deg"] = max(worst["rotation_deg"], angle)

    return {
        "samples": samples,
        "worst_position_error_m": worst["position_m"],
        "worst_rotation_error_deg": worst["rotation_deg"],
        "worst_case": worst_case,
        "passed": worst["position_m"] <= 1e-6 and worst["rotation_deg"] <= 1e-3,
    }


def _wheel_geometry_check(corrected: RobotModel, model: mujoco.MjModel) -> dict[str, Any]:
    details = {}
    ok = True
    for name in ("right_wheel", "left_wheel"):
        link = corrected.links[name]
        visual_offset = float(np.linalg.norm(link.visuals[0].xyz))
        collision_offset = float(np.linalg.norm(link.collisions[0].xyz))
        joint = next(j for j in corrected.joints.values() if j.child == name)
        axis = np.asarray(joint.axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        axis_alignment = float(abs(np.dot(axis, [0.0, 0.0, 1.0])))
        radius = float(link.collisions[0].geometry.size[0])
        passed = visual_offset <= 1e-6 and collision_offset <= 1e-6 and axis_alignment >= 1.0 - 1e-9
        ok = ok and passed
        details[name] = {
            "visual_offset_m": visual_offset,
            "collision_offset_m": collision_offset,
            "joint_axis_dot_link_z": axis_alignment,
            "collision_radius_m": radius,
            "passed": passed,
        }
    return {"passed": ok, "per_wheel": details}


def _lowest_point_z(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> float | None:
    geom_type = int(model.geom_type[geom_id])
    position = data.geom_xpos[geom_id]
    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    size = model.geom_size[geom_id]

    if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
        return float(position[2] - np.sum(np.abs(rotation[2, :]) * size[:3]))
    if geom_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        axis = rotation[:, 2]
        half_length = size[1]
        radius = size[0]
        return float(
            position[2]
            - half_length * abs(axis[2])
            - radius * float(np.sqrt(max(0.0, 1.0 - axis[2] ** 2)))
        )
    if geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        axis = rotation[:, 2]
        return float(position[2] - size[1] * abs(axis[2]) - size[0])
    if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        return float(position[2] - size[0])
    return None


def spawn_geometry_check(
    model: mujoco.MjModel, config: Config, paths: MjcfPaths
) -> dict[str, Any]:
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    wheel_geoms = []
    all_geoms = []
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) == 0:
            continue  # world/floor
        all_geoms.append(geom_id)
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_id]))
        if body_name and body_name.endswith("_wheel"):
            wheel_geoms.append(geom_id)

    wheel_lowest = {geom_id: _lowest_point_z(model, data, geom_id) for geom_id in wheel_geoms}
    wheel_lowest = {k: v for k, v in wheel_lowest.items() if v is not None}
    robot_lowest = [
        value for geom_id in all_geoms if (value := _lowest_point_z(model, data, geom_id)) is not None
    ]
    clearance = float(config.spawn["spawn"]["wheel_clearance_m"])
    tolerance = float(config.spawn["checks"]["wheel_clearance_tolerance_m"])

    min_wheel = min(wheel_lowest.values()) if wheel_lowest else float("nan")
    root_z = float(data.qpos[2])
    expected_root_z = paths.spawn_z

    return {
        "root_z_m": root_z,
        "expected_root_z_m": expected_root_z,
        "wheel_lowest_points_m": sorted(wheel_lowest.values()),
        "min_wheel_clearance_m": min_wheel,
        "expected_clearance_m": clearance,
        "lowest_robot_point_m": min(robot_lowest) if robot_lowest else float("nan"),
        "contacts_at_spawn": int(data.ncon),
        "passed": bool(
            abs(root_z - expected_root_z) <= 1e-9
            and abs(min_wheel - clearance) <= tolerance
            and min(robot_lowest) >= -1e-9
            and int(data.ncon) == 0
        ),
    }


def physical_parity_check(visual: mujoco.MjModel, train: mujoco.MjModel, corrected: RobotModel) -> dict[str, Any]:
    details: dict[str, Any] = {}
    ok = True
    for name, link in corrected.links.items():
        body_visual = _body_id(visual, name)
        body_train = _body_id(train, name)
        mass_delta = abs(float(visual.body_mass[body_visual]) - float(train.body_mass[body_train]))
        ipos_delta = float(
            np.abs(visual.body_ipos[body_visual] - train.body_ipos[body_train]).max()
        )
        inertia_delta = float(
            np.abs(visual.body_inertia[body_visual] - train.body_inertia[body_train]).max()
        )
        reference_mass = 0.0 if link.inertial is None else link.inertial.mass
        # MuJoCo stores body_mass as float32, so compare with an explicit
        # relative tolerance instead of demanding bit equality.
        mass_tolerance = max(1e-6 * abs(reference_mass), 1e-9)
        mass_matches_urdf = abs(float(train.body_mass[body_train]) - reference_mass) <= mass_tolerance
        passed = (
            mass_delta <= mass_tolerance
            and ipos_delta <= 1e-9
            and inertia_delta <= 1e-9 * max(1.0, float(np.abs(train.body_inertia[body_train]).max()))
            and mass_matches_urdf
        )
        ok = ok and passed
        details[name] = {
            "mass_delta_kg": mass_delta,
            "mass_tolerance_kg": mass_tolerance,
            "ipos_delta_m": ipos_delta,
            "inertia_delta": inertia_delta,
            "mass_matches_corrected_urdf": mass_matches_urdf,
            "passed": passed,
        }

    joint_details = {}
    for name in corrected.joints:
        joint_visual = _joint_id(visual, name)
        joint_train = _joint_id(train, name)
        axis_delta = float(np.abs(visual.jnt_axis[joint_visual] - train.jnt_axis[joint_train]).max())
        range_delta = float(np.abs(visual.jnt_range[joint_visual] - train.jnt_range[joint_train]).max())
        limited_equal = bool(visual.jnt_limited[joint_visual] == train.jnt_limited[joint_train])
        passed = axis_delta <= 1e-12 and range_delta <= 1e-12 and limited_equal
        ok = ok and passed
        joint_details[name] = {
            "axis_delta": axis_delta,
            "range_delta": range_delta,
            "limited_equal": limited_equal,
            "passed": passed,
        }

    return {
        "passed": ok,
        "bodies": details,
        "joints": joint_details,
        "total_mass_kg": float(model_total_mass(train)),
    }


def model_total_mass(model: mujoco.MjModel) -> float:
    return float(sum(model.body_mass[body_id] for body_id in range(1, model.nbody)))


def inertia_check(model: mujoco.MjModel, corrected: RobotModel) -> dict[str, Any]:
    per_link = {}
    ok = True
    for name, link in corrected.links.items():
        if link.inertial is None:
            continue
        valid, note = inertia_is_physically_valid(link.inertial.inertia)
        body_id = _body_id(model, name)
        principal = principal_inertia(
            np.array(
                [
                    [model.body_inertia[body_id][0], 0.0, 0.0],
                    [0.0, model.body_inertia[body_id][1], 0.0],
                    [0.0, 0.0, model.body_inertia[body_id][2]],
                ]
            )
        )
        compiled_valid, compiled_note = inertia_is_physically_valid(np.diag(model.body_inertia[body_id]))
        ok = ok and valid and compiled_valid
        per_link[name] = {
            "corrected_principal_moments": principal_inertia(link.inertial.inertia).tolist(),
            "compiled_principal_moments": principal.tolist(),
            "valid": valid and compiled_valid,
            "note": note if valid else compiled_note,
        }
    return {"passed": ok, "per_link": per_link}


def validate(config: Config, assets: CorrectedAssets, paths: MjcfPaths) -> dict[str, Any]:
    visual_model = mujoco.MjModel.from_xml_path(str(paths.visual))
    train_model = mujoco.MjModel.from_xml_path(str(paths.train))
    corrected = assets.robot

    checks: dict[str, Any] = {}
    spring_count = config.spring_count
    expected_nu = len(config.joint_order) + spring_count
    checks["model_dimensions"] = {
        "passed": train_model.nq == 13
        and train_model.nv == 12
        and train_model.nu == expected_nu
        and train_model.nbody == len(corrected.links) + 1
        and train_model.ntendon == spring_count,
        "nq": int(train_model.nq),
        "nv": int(train_model.nv),
        "nu": int(train_model.nu),
        "ntendon": int(train_model.ntendon),
        "nbody": int(train_model.nbody),
        "expected": {
            "nq": 13,
            "nv": 12,
            "nu": expected_nu,
            "ntendon": spring_count,
            "nbody": len(corrected.links) + 1,
        },
        "note": "nu counts actuators, not policy actions (policy stays 6)",
    }
    checks["fk_parity_train"] = fk_parity_check(train_model, corrected, config, samples=200)
    checks["fk_parity_visual"] = fk_parity_check(visual_model, corrected, config, samples=20)
    checks["wheel_geometry"] = _wheel_geometry_check(corrected, train_model)
    checks["physical_parity_visual_vs_train"] = physical_parity_check(visual_model, train_model, corrected)
    checks["inertia"] = inertia_check(train_model, corrected)
    checks["spawn_geometry"] = spawn_geometry_check(train_model, config, paths)

    total_mass = model_total_mass(train_model)
    mass_reference = corrected.total_mass()
    checks["total_mass"] = {
        "passed": abs(total_mass - mass_reference) <= max(1e-6 * mass_reference, 1e-9),
        "compiled_kg": total_mass,
        "urdf_kg": mass_reference,
        "tolerance_note": "MuJoCo body_mass is float32",
    }
    checks["gas_spring"] = {
        "passed": (
            (not config.gas_spring_configured and train_model.ntendon == 0)
            or (config.gas_spring_configured and train_model.ntendon == config.spring_count)
        ),
        "status": (
            "not_generated_missing_geometry_and_force"
            if not config.gas_spring_configured
            else f"generated_{config.gas_spring_profile}"
        ),
        "tendon_count": int(train_model.ntendon),
        "profile": config.gas_spring_profile,
        "stroke_verified": config.gas_spring_stroke_verified,
        "note": "P3 still needs measured anchors + stroke; reference_300n is not calibrated",
    }

    report = {
        "stage": "P1",
        "status": "passed" if all(check["passed"] for check in checks.values()) else "failed",
        "models": {
            "visual": {"path": str(paths.visual), "nmesh": int(visual_model.nmesh)},
            "train": {"path": str(paths.train), "nmesh": int(train_model.nmesh)},
        },
        "spawn_z_m": paths.spawn_z,
        "wheel_axes_z_m": paths.wheel_axes_z,
        "checks": checks,
        "corrected_asset_report": str(
            Path(__file__).resolve().parents[3] / "assets" / "corrected" / "transform_report.json"
        ),
    }
    return report


def write_validation_report(report: dict[str, Any], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path

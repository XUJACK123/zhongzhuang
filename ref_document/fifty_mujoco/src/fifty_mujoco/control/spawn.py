"""Spawn contract (doc section 10.4-10.6).

One ``SpawnSpec`` feeds the MJCF keyframe, the CPU environment, the later GPU
environment and the ONNX playback.  The source->training rotation is applied by
the asset builder only; nothing downstream is allowed to rotate again.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..config import Config
from ..envs.cpu_env import FiftyCpuEnv


@dataclass(frozen=True)
class SpawnSpec:
    frame: str
    root_quat_wxyz: np.ndarray
    root_xy_m: np.ndarray
    root_z_mode: str
    wheel_clearance_m: float
    joint_pose_source: str
    initial_linear_velocity_m_s: np.ndarray
    initial_angular_velocity_rad_s: np.ndarray


def load_spawn_spec(config: Config) -> SpawnSpec:
    spawn = config.spawn["spawn"]
    quat = np.asarray(spawn["root_quat_wxyz"], dtype=float)
    return SpawnSpec(
        frame=str(spawn["frame"]),
        root_quat_wxyz=quat / np.linalg.norm(quat),
        root_xy_m=np.asarray(spawn["root_xy_m"], dtype=float),
        root_z_mode=str(spawn["root_z_mode"]),
        wheel_clearance_m=float(spawn["wheel_clearance_m"]),
        joint_pose_source=str(spawn["joint_pose_source"]),
        initial_linear_velocity_m_s=np.asarray(spawn["initial_linear_velocity_m_s"], dtype=float),
        initial_angular_velocity_rad_s=np.asarray(spawn["initial_angular_velocity_rad_s"], dtype=float),
    )


def _lowest_point_z(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> float | None:
    geom_type = int(model.geom_type[geom_id])
    position = data.geom_xpos[geom_id]
    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    size = model.geom_size[geom_id]
    if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
        return float(position[2] - np.sum(np.abs(rotation[2, :]) * size[:3]))
    if geom_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        axis = rotation[:, 2]
        return float(
            position[2] - size[1] * abs(axis[2]) - size[0] * float(np.sqrt(max(0.0, 1.0 - axis[2] ** 2)))
        )
    if geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        axis = rotation[:, 2]
        return float(position[2] - size[1] * abs(axis[2]) - size[0])
    if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        return float(position[2] - size[0])
    return None


def wheel_bottom_points(env: FiftyCpuEnv) -> dict[str, float]:
    result: dict[str, float] = {}
    for body_name, geom_ids in env.wheel_geom_ids.items():
        values = [
            value
            for geom_id in geom_ids
            if (value := _lowest_point_z(env.model, env.data, geom_id)) is not None
        ]
        if values:
            result[body_name] = min(values)
    return result


def resolve_spawn_height(env: FiftyCpuEnv, spec: SpawnSpec) -> tuple[float, dict[str, float]]:
    """Put the root at rest with the lowest wheel bottom at ``clearance``."""
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], 0.0]))
    bottoms = wheel_bottom_points(env)
    if not bottoms:
        raise ValueError("no wheel collision geometry found; cannot resolve spawn height")
    root_z = spec.wheel_clearance_m - min(bottoms.values())
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], root_z]))
    return float(root_z), wheel_bottom_points(env)


def _top_marker_local(config: Config) -> np.ndarray:
    configured = config.spawn["spawn"].get("top_marker_local_m")
    if configured is not None:
        return np.asarray(configured, dtype=float)
    report_path = Path(__file__).resolve().parents[3] / "assets" / "corrected" / "transform_report.json"
    if not report_path.is_file():
        raise FileNotFoundError(
            f"{report_path} missing: run `build-assets` first, or set spawn.top_marker_local_m"
        )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    return np.asarray(report["base_geometry"]["top_point_local_m"], dtype=float)


def audit_spawn(env: FiftyCpuEnv, config: Config, spec: SpawnSpec) -> dict[str, Any]:
    thresholds = config.spawn["checks"]
    env.reset(root_position=np.array([spec.root_xy_m[0], spec.root_xy_m[1], 0.0]))
    root_z, _ = resolve_spawn_height(env, spec)

    state = env.state()
    quat = np.asarray(state["root_quat_wxyz"], dtype=float)
    rotation = np.asarray(state["root_rotation"], dtype=float)

    up_dot = float(rotation[2, 2])  # base +Z projected on world +Z
    projected_gravity = rotation.T @ np.array([0.0, 0.0, -1.0])

    marker_local = _top_marker_local(config)
    marker_world = np.asarray(state["root_position_m"]) + rotation @ marker_local

    wheel_axes = {
        body: env.data.xpos[body_id].copy()
        for body, body_id in env.body_ids.items()
        if body.endswith("_wheel")
    }
    wheel_bottoms = wheel_bottom_points(env)

    hip_origins = {
        name: env.data.xanchor[joint_id].copy()
        for name, joint_id in env.joint_ids.items()
        if env.config.joint_group(name) == "hip"
    }

    robot_lowest = [
        value
        for geom_id in env.robot_geom_ids
        if (value := _lowest_point_z(env.model, env.data, geom_id)) is not None
    ]

    wheel_axle_z = {name: float(position[2]) for name, position in wheel_axes.items()}
    clearance_ok = all(
        abs(value - spec.wheel_clearance_m) <= float(thresholds["wheel_clearance_tolerance_m"])
        for value in wheel_bottoms.values()
    )
    wheel_delta = (
        abs(wheel_bottoms["right_wheel"] - wheel_bottoms["left_wheel"])
        if {"right_wheel", "left_wheel"} <= set(wheel_bottoms)
        else float("nan")
    )

    checks = {
        "quaternion_norm": {
            "value": float(np.linalg.norm(quat)),
            "error": float(abs(np.linalg.norm(quat) - 1.0)),
            "passed": float(abs(np.linalg.norm(quat) - 1.0))
            <= float(thresholds["quaternion_norm_tolerance"]),
        },
        "base_up_dot": {
            "value": up_dot,
            "passed": up_dot >= float(thresholds["min_base_up_dot"]),
        },
        "projected_gravity": {
            "value": projected_gravity.tolist(),
            "error_vs_minus_z": float(np.linalg.norm(projected_gravity - np.array([0.0, 0.0, -1.0]))),
            "passed": float(np.linalg.norm(projected_gravity - np.array([0.0, 0.0, -1.0])))
            <= float(thresholds["projected_gravity_tolerance"]),
        },
        "top_marker_above_wheel_axle": {
            "marker_world_m": marker_world.tolist(),
            "marker_z_m": float(marker_world[2]),
            "max_wheel_axle_z_m": max(wheel_axle_z.values()),
            "passed": float(marker_world[2]) > max(wheel_axle_z.values()),
            "marker_source": "spawn.top_marker_local_m or corrected base mesh top vertex",
        },
        "wheel_clearance": {
            "per_wheel_m": wheel_bottoms,
            "expected_m": spec.wheel_clearance_m,
            "passed": clearance_ok,
        },
        "left_right_wheel_height": {
            "delta_m": wheel_delta,
            "passed": wheel_delta <= float(thresholds["left_right_wheel_delta_tolerance_m"]),
        },
        "no_ground_penetration": {
            "lowest_robot_point_m": min(robot_lowest) if robot_lowest else None,
            "contacts": int(env.data.ncon),
            "passed": bool(robot_lowest) and min(robot_lowest) >= -1e-9 and int(env.data.ncon) == 0,
        },
        "source_rotation_applied_once": {
            "root_quat_wxyz": state["root_quat_wxyz"],
            "identity_expected": [1.0, 0.0, 0.0, 0.0],
            "passed": bool(np.allclose(state["root_quat_wxyz"], [1.0, 0.0, 0.0, 0.0], atol=1e-9)),
        },
    }

    return {
        "stage": "P2",
        "status": "passed" if all(check["passed"] for check in checks.values()) else "failed",
        "profile": "geometry_only",
        "xml_path": str(env.xml_path.resolve()),
        "xml_sha256": env.model_summary()["xml_sha256"],
        "spawn_z_m": root_z,
        "checks": checks,
        "frames": {
            "root_position_m": state["root_position_m"],
            "root_quat_wxyz": state["root_quat_wxyz"],
            "hips_world_m": {name: origin.tolist() for name, origin in hip_origins.items()},
            "wheel_axles_world_m": {name: position.tolist() for name, position in wheel_axes.items()},
            "wheel_bottoms_world_z_m": wheel_bottoms,
            "lowest_robot_point_m": min(robot_lowest) if robot_lowest else None,
            "top_marker_world_m": marker_world.tolist(),
        },
        "joint_pose": {
            "default_angles_rad": config.default_angles,
            "limits_rad": {k: (None if v is None else list(v)) for k, v in config.joint_limits.items()},
        },
        "gas_spring": {
            "configured": config.gas_spring_configured,
            "status": "not_generated_missing_geometry_and_force"
            if not config.gas_spring_configured
            else "configured",
            "lengths_m": None,
            "note": "no passive spring force exists in this model; P3/P4 remain unpassed",
        },
    }

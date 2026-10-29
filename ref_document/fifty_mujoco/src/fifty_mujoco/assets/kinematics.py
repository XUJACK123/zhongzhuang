"""Forward kinematics over a parsed URDF model.

The whole point of this module is that the source->training transform is
computed from *two full joint trees evaluated at the same physical assembly
pose*, instead of from hand-composed per-link rotations.
"""

from __future__ import annotations

from collections import defaultdict

import numpy as np

from ..math3d import Transform, axis_angle
from .urdf_io import Joint, RobotModel


def topological_joints(robot: RobotModel) -> list[Joint]:
    """Joints ordered so that every parent pose is known before its children."""
    children = robot.child_map()
    ordered: list[Joint] = []
    stack: list[str] = [robot.root_link]
    visited: set[str] = set()
    while stack:
        link = stack.pop()
        if link in visited:
            continue
        visited.add(link)
        for joint in children.get(link, []):
            ordered.append(joint)
            stack.append(joint.child)
    if len(ordered) != len(robot.joints):
        missing = sorted(set(robot.joints) - {j.name for j in ordered})
        raise ValueError(f"joint tree is disconnected or cyclic; unreachable: {missing}")
    return ordered


def link_poses(robot: RobotModel, q: dict[str, float] | None = None) -> dict[str, Transform]:
    """World (root-link) pose of every link for joint configuration ``q``."""
    q = q or {}
    poses: dict[str, Transform] = {robot.root_link: Transform.identity()}
    for joint in topological_joints(robot):
        parent_pose = poses[joint.parent]
        joint_pose = parent_pose @ joint.origin()
        if joint.type == "fixed":
            poses[joint.child] = joint_pose
        elif joint.type in {"revolute", "continuous"}:
            angle = float(q.get(joint.name, 0.0))
            poses[joint.child] = joint_pose @ Transform(axis_angle(joint.axis, angle), np.zeros(3))
        elif joint.type == "prismatic":
            offset = np.asarray(joint.axis, dtype=float)
            offset = offset / np.linalg.norm(offset) * float(q.get(joint.name, 0.0))
            poses[joint.child] = Transform(joint_pose.R, joint_pose.t + joint_pose.rotate(offset))
        else:
            raise ValueError(f"unsupported joint type {joint.type!r} on {joint.name}")
    return poses


def joint_axis_world(robot: RobotModel, q: dict[str, float] | None, name: str) -> np.ndarray:
    """Rotation axis of joint ``name`` expressed in the root-link frame."""
    q = q or {}
    poses = link_poses(robot, q)
    joint = robot.joints[name]
    parent_pose = poses[joint.parent]
    return parent_pose.rotate(joint.origin().rotate(joint.axis))


def joint_origin_world(robot: RobotModel, q: dict[str, float] | None, name: str) -> np.ndarray:
    q = q or {}
    poses = link_poses(robot, q)
    joint = robot.joints[name]
    return poses[joint.parent].apply(joint.xyz)


def relative_pose(
    robot_from: RobotModel,
    q_from: dict[str, float] | None,
    robot_to: RobotModel,
    q_to: dict[str, float] | None,
    link: str,
    to_from_root: Transform,
) -> Transform:
    """Rigid transform mapping the ``link`` frame of one model onto the other.

    ``T_to<-from = inv(T_to_root(link, q_to)) @ T_to_from_root @ T_from_root(link, q_from)``

    Both models are assumed to describe the same physical assembly, so this is
    the only transform that may ever be applied to that link's visual mesh,
    centre of mass and inertia tensor.
    """
    pose_from = link_poses(robot_from, q_from)[link]
    pose_to = link_poses(robot_to, q_to)[link]
    return pose_to.inverse() @ to_from_root @ pose_from


def joint_value_consistency(
    robot_from: RobotModel,
    robot_to: RobotModel,
    q_from: dict[str, float] | None,
    q_to: dict[str, float] | None,
    to_from_root: Transform,
    names: list[str],
) -> dict[str, dict]:
    """Compare joint axes/origins of two models at the same physical pose."""
    poses_from = link_poses(robot_from, q_from)
    poses_to = link_poses(robot_to, q_to)
    result: dict[str, dict] = {}
    for name in names:
        joint_from = robot_from.joints[name]
        joint_to = robot_to.joints[name]
        origin_from_world = poses_from[joint_from.parent].apply(joint_from.xyz)
        origin_to_world = poses_to[joint_to.parent].apply(joint_to.xyz)
        origin_from_in_to = to_from_root.apply(origin_from_world)
        axis_from_world = poses_from[joint_from.parent].rotate(joint_from.origin().rotate(joint_from.axis))
        axis_to_world = poses_to[joint_to.parent].rotate(joint_to.origin().rotate(joint_to.axis))
        axis_from_in_to = to_from_root.rotate(axis_from_world)
        result[name] = {
            "origin_error_m": float(np.linalg.norm(origin_from_in_to - origin_to_world)),
            "axis_dot": float(np.dot(axis_from_in_to, axis_to_world)),
            "axis_angle_deg": float(
                np.degrees(
                    np.arccos(np.clip(np.dot(axis_from_in_to, axis_to_world), -1.0, 1.0))
                )
            ),
        }
    return result


def duplicate_parent_links(robot: RobotModel) -> dict[str, int]:
    """Sanity helper: number of joints per parent link."""
    counts: dict[str, int] = defaultdict(int)
    for joint in robot.joints.values():
        counts[joint.parent] += 1
    return dict(counts)

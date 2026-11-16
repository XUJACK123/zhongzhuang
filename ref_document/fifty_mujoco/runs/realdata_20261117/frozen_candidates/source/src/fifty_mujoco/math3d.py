"""Rigid-body math helpers.

Convention notes
----------------
* URDF ``rpy`` means fixed-axis roll-pitch-yaw, i.e. ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.
* A :class:`Transform` maps coordinates expressed in frame *A* into frame *B*
  when used as ``T_B_from_A``.  Composition ``T_c = T_b @ T_a`` means "apply
  ``T_a`` first".
* Inertia matrices are expressed in a frame whose origin is the centre of mass.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

EPS = 1e-12


def rot_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def rot_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def rpy_to_mat(rpy: Iterable[float]) -> np.ndarray:
    roll, pitch, yaw = (float(v) for v in rpy)
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def mat_to_rpy(matrix: np.ndarray) -> np.ndarray:
    """Inverse of :func:`rpy_to_mat`, with explicit gimbal-lock handling."""
    m = np.asarray(matrix, dtype=float)
    sy = -float(m[2, 0])
    sy = max(-1.0, min(1.0, sy))
    pitch = np.arcsin(sy)
    if abs(sy) < 1.0 - 1e-9:
        roll = np.arctan2(m[2, 1], m[2, 2])
        yaw = np.arctan2(m[1, 0], m[0, 0])
    else:  # pitch = +-90 deg: roll and yaw are not separable, pick roll = 0
        roll = 0.0
        yaw = np.arctan2(-m[0, 1], m[1, 1])
    return np.array([roll, pitch, yaw], dtype=float)


def axis_angle(axis: Iterable[float], angle: float) -> np.ndarray:
    """Rodrigues rotation about an arbitrary (not necessarily unit) axis."""
    a = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(a))
    if norm < EPS:
        raise ValueError("rotation axis must be non-zero")
    a = a / norm
    c, s = np.cos(angle), np.sin(angle)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]], dtype=float)
    return np.eye(3) + s * k + (1.0 - c) * (k @ k)


def skew(v: Iterable[float]) -> np.ndarray:
    x, y, z = (float(c) for c in v)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def is_rotation(matrix: np.ndarray, atol: float = 1e-9) -> bool:
    m = np.asarray(matrix, dtype=float)
    if m.shape != (3, 3):
        return False
    if not np.allclose(m.T @ m, np.eye(3), atol=atol):
        return False
    return bool(abs(np.linalg.det(m) - 1.0) <= 1e-6)


@dataclass(frozen=True)
class Transform:
    """Rigid transform ``p_B = R @ p_A + t``."""

    R: np.ndarray
    t: np.ndarray

    # ---------------------------------------------------------------- builders
    @staticmethod
    def identity() -> "Transform":
        return Transform(np.eye(3), np.zeros(3))

    @staticmethod
    def from_xyz_rpy(xyz: Iterable[float], rpy: Iterable[float]) -> "Transform":
        return Transform(rpy_to_mat(rpy), np.asarray(list(xyz), dtype=float))

    @staticmethod
    def from_matrix4(m: np.ndarray) -> "Transform":
        m = np.asarray(m, dtype=float)
        return Transform(m[:3, :3].copy(), m[:3, 3].copy())

    # ---------------------------------------------------------------- operators
    def __matmul__(self, other: "Transform") -> "Transform":
        return Transform(self.R @ other.R, self.R @ other.t + self.t)

    def inverse(self) -> "Transform":
        rt = self.R.T
        return Transform(rt, -(rt @ self.t))

    def apply(self, point: Iterable[float]) -> np.ndarray:
        return self.R @ np.asarray(point, dtype=float) + self.t

    def rotate(self, vector: Iterable[float]) -> np.ndarray:
        return self.R @ np.asarray(vector, dtype=float)

    # ---------------------------------------------------------------- exporters
    def to_matrix4(self) -> np.ndarray:
        m = np.eye(4)
        m[:3, :3] = self.R
        m[:3, 3] = self.t
        return m

    def to_xyz_rpy(self) -> tuple[np.ndarray, np.ndarray]:
        return self.t.copy(), mat_to_rpy(self.R)

    # ------------------------------------------------------------- validations
    def assert_valid(self, name: str = "transform", atol: float = 1e-9) -> None:
        if not is_rotation(self.R, atol=atol):
            raise ValueError(f"{name}: rotation block is not a proper rotation matrix")
        if not np.all(np.isfinite(self.t)):
            raise ValueError(f"{name}: translation contains non-finite values")

    def max_delta(self, other: "Transform") -> float:
        return float(max(np.abs(self.R - other.R).max(), np.abs(self.t - other.t).max()))

    def as_dict(self) -> dict:
        return {"R": self.R.tolist(), "t": self.t.tolist()}


def sym_6_to_matrix(values: dict) -> np.ndarray:
    """URDF/MJCF symmetric inertia components -> 3x3 matrix."""
    ixx = float(values["ixx"])
    iyy = float(values["iyy"])
    izz = float(values["izz"])
    ixy = float(values.get("ixy", 0.0))
    ixz = float(values.get("ixz", 0.0))
    iyz = float(values.get("iyz", 0.0))
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)


def matrix_to_sym_6(matrix: np.ndarray) -> dict:
    m = np.asarray(matrix, dtype=float)
    return {
        "ixx": float(m[0, 0]),
        "ixy": float(m[0, 1]),
        "ixz": float(m[0, 2]),
        "iyy": float(m[1, 1]),
        "iyz": float(m[1, 2]),
        "izz": float(m[2, 2]),
    }


def principal_inertia(matrix: np.ndarray) -> np.ndarray:
    """Eigenvalues of an inertia tensor, sorted ascending."""
    m = np.asarray(matrix, dtype=float)
    m = 0.5 * (m + m.T)
    return np.sort(np.linalg.eigvalsh(m))


def inertia_is_physically_valid(matrix: np.ndarray, atol: float = 1e-12) -> tuple[bool, str]:
    """Positive definite and triangle inequality on principal moments."""
    principal = principal_inertia(matrix)
    if principal[0] <= atol:
        return False, f"not positive definite: principal moments {principal}"
    a, b, c = principal
    if a + b < c - 1e-9:
        return False, f"violates triangle inequality: {principal}"
    return True, "ok"


def rotation_between(a: Iterable[float], b: Iterable[float]) -> np.ndarray:
    """Minimal rotation taking unit vector ``a`` onto unit vector ``b``."""
    va = np.asarray(a, dtype=float)
    vb = np.asarray(b, dtype=float)
    va = va / np.linalg.norm(va)
    vb = vb / np.linalg.norm(vb)
    cross = np.cross(va, vb)
    dot = float(np.dot(va, vb))
    if np.linalg.norm(cross) < 1e-12:
        if dot > 0:
            return np.eye(3)
        # 180 degrees: pick any perpendicular axis
        helper = np.array([1.0, 0.0, 0.0])
        if abs(va[0]) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])
        axis = np.cross(va, helper)
        return axis_angle(axis, np.pi)
    axis = cross / np.linalg.norm(cross)
    return axis_angle(axis, float(np.arccos(max(-1.0, min(1.0, dot)))))

def mat_to_quat_wxyz(matrix: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion in MuJoCo ``wxyz`` order."""
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    quat = np.array([w, x, y, z], dtype=float)
    quat /= np.linalg.norm(quat)
    # MuJoCo normalises to w >= 0 internally; match that for stable comparisons.
    if quat[0] < 0.0:
        quat = -quat
    return quat

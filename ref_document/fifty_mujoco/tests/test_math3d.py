"""Rigid-transform math: conventions and single-application guarantees."""

from __future__ import annotations

import numpy as np
import pytest

from fifty_mujoco.math3d import (
    Transform,
    axis_angle,
    inertia_is_physically_valid,
    is_rotation,
    mat_to_quat_wxyz,
    mat_to_rpy,
    rpy_to_mat,
)


@pytest.mark.parametrize(
    "rpy",
    [
        (0.0, 0.0, 0.0),
        (np.pi, 0.0, np.pi / 2),
        (-np.pi, 0.0, 1.0861711),
        (0.0, np.pi / 2, 0.0),
        (0.3, -0.7, 1.2),
    ],
)
def test_rpy_round_trip(rpy):
    matrix = rpy_to_mat(rpy)
    assert is_rotation(matrix)
    recovered = rpy_to_mat(mat_to_rpy(matrix))
    assert np.allclose(recovered, matrix, atol=1e-12)


def test_base_transform_matches_recorded_rotation():
    from fifty_mujoco.math3d import Transform as T

    # Recorded source->training rotation from model_parameters.json.
    rotation = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    quat = mat_to_quat_wxyz(rotation)
    assert np.allclose(quat, [0.0, 0.7071067811865476, 0.7071067811865476, 0.0], atol=1e-12)
    transform = T(rotation, np.zeros(3))
    assert np.allclose(np.linalg.det(transform.R), 1.0)


def test_transform_inverse_and_composition():
    transform = Transform.from_xyz_rpy([0.1, -0.2, 0.3], [0.4, 0.5, 0.6])
    identity = transform @ transform.inverse()
    assert np.allclose(identity.R, np.eye(3), atol=1e-12)
    assert np.allclose(identity.t, np.zeros(3), atol=1e-12)

    point = np.array([1.0, 2.0, 3.0])
    assert np.allclose(transform.inverse().apply(transform.apply(point)), point, atol=1e-12)


def test_axis_angle_is_proper_rotation():
    for axis in ([0, 0, 1], [1, 0, 0], [0.3, -0.4, 0.5]):
        matrix = axis_angle(axis, 1.234)
        assert is_rotation(matrix)
        assert np.isclose(np.linalg.det(matrix), 1.0)


def test_applying_a_transform_twice_is_detectable():
    """Guards the 'transform applied twice' class of bug."""
    transform = Transform.from_xyz_rpy([0.24, 0.045, 0.0], [np.pi, 0.0, 0.5])
    once = transform.t
    twice = (transform @ transform).t
    assert np.linalg.norm(twice - once) > 0.1


def test_inertia_validity_flags_violations():
    good = np.diag([1.0, 1.0, 1.0])
    assert inertia_is_physically_valid(good)[0]
    bad = np.diag([1.0, 1.0, 5.0])
    assert not inertia_is_physically_valid(bad)[0]
    negative = np.diag([-1.0, 1.0, 1.0])
    assert not inertia_is_physically_valid(negative)[0]

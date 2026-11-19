"""Rotation helpers: these must be right, or everything built on them is wrong."""

import math

import numpy as np
import pytest

from backend.sim.frames import (
    body_up_in_world,
    integrate_quat,
    quat_from_euler,
    quat_identity,
    quat_to_euler,
    quat_to_rotation_matrix,
)


@pytest.mark.parametrize("roll,pitch,yaw", [(0.1, -0.2, 0.3), (-0.5, 0.4, -2.5), (0.0, 0.0, math.pi / 2)])
def test_euler_round_trip(roll, pitch, yaw):
    assert np.allclose(quat_to_euler(quat_from_euler(roll, pitch, yaw)), (roll, pitch, yaw))


def test_rotation_matrix_is_a_pure_rotation():
    r = quat_to_rotation_matrix(quat_from_euler(0.3, -0.7, 1.1))
    assert np.allclose(r @ r.T, np.eye(3))
    assert np.isclose(np.linalg.det(r), 1.0)


def test_body_up_matches_rotation_matrix_column():
    q = quat_from_euler(0.3, -0.7, 1.1)
    assert np.allclose(body_up_in_world(q), quat_to_rotation_matrix(q)[:, 2])


def test_sign_conventions():
    """Positive yaw turns the nose from east toward north; positive pitch puts the nose down;
    positive roll puts the right side down."""
    forward = np.array([1.0, 0.0, 0.0])
    left = np.array([0.0, 1.0, 0.0])
    assert np.allclose(quat_to_rotation_matrix(quat_from_euler(0, 0, math.pi / 2)) @ forward, [0, 1, 0], atol=1e-12)
    assert (quat_to_rotation_matrix(quat_from_euler(0, 0.2, 0)) @ forward)[2] < 0
    right = -left
    assert (quat_to_rotation_matrix(quat_from_euler(0.2, 0, 0)) @ right)[2] < 0


def test_integrating_a_constant_yaw_rate():
    q = quat_identity()
    for _ in range(500):  # 1 s at 90°/s
        q = integrate_quat(q, np.array([0.0, 0.0, math.pi / 2]), 0.002)
    assert np.isclose(quat_to_euler(q)[2], math.pi / 2, atol=1e-9)
    assert np.isclose(np.linalg.norm(q), 1.0)

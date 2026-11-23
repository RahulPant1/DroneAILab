"""Coordinate frames and rotation helpers.

WORLD frame (ENU): x = east, y = north, z = UP. The ground is the plane z = 0.
BODY frame (FLU), fixed to the drone: x = forward, y = left, z = up through the propellers.

Attitude is stored as a unit quaternion q = [w, x, y, z]. It ROTATES body-frame
vectors into the world frame: v_world = R(q) @ v_body.

Euler angles (shown in the UI) use the yaw -> pitch -> roll order (ZYX):
  roll  = rotation about body x (forward). Positive roll: right side goes DOWN.
  pitch = rotation about body y (left).    Positive pitch: nose goes DOWN.
  yaw   = rotation about world z (up).     Positive yaw: turns LEFT (counter-clockwise from above).
These are the ROS / right-hand-rule signs; with z up, "positive pitch = nose down" surprises
many people, so the UI labels it explicitly.
"""

from __future__ import annotations

import math

import numpy as np


def quat_identity() -> np.ndarray:
    """A level drone facing east: no rotation at all."""
    return np.array([1.0, 0.0, 0.0, 0.0])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Rescale to length 1. Small rounding errors pile up each step, so we do this every step."""
    return q / math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """The opposite rotation (for unit quaternions, conjugate == inverse)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Combine two rotations: rotate by b first, then by a (Hamilton product a ⊗ b)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """3x3 matrix R with v_world = R @ v_body. Column 2 is the body 'up' axis in world coordinates."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def body_up_in_world(q: np.ndarray) -> np.ndarray:
    """Direction the propellers push, in world coordinates (column 2 of R, without building all of R)."""
    w, x, y, z = q
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def quat_from_euler(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """Quaternion for 'yaw, then pitch, then roll' (ZYX order)."""
    cr, sr = math.cos(roll_rad / 2), math.sin(roll_rad / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    cy, sy = math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quat_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    """(roll, pitch, yaw) in radians, ZYX order. Pitch is limited to ±90°."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def integrate_quat(q: np.ndarray, rate_body_rad_s: np.ndarray, dt_s: float) -> np.ndarray:
    """Turn the attitude by the body spin rate for dt seconds.

    Over one tiny step the spin is treated as constant, so the drone turns by
    angle |rate|*dt about the rate's axis. That small turn is applied in the body frame.
    """
    wx, wy, wz = rate_body_rad_s
    rate = math.sqrt(wx * wx + wy * wy + wz * wz)
    angle = rate * dt_s
    if angle < 1e-12:
        return q
    s = math.sin(angle / 2) / rate
    dq = np.array([math.cos(angle / 2), wx * s, wy * s, wz * s])
    return quat_normalize(quat_multiply(q, dq))


def cross3(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Cross product for 3-vectors (faster than np.cross for tiny arrays)."""
    return np.array(
        [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    )


def quat_from_rotation_matrix(r: np.ndarray) -> np.ndarray:
    """The quaternion for a rotation matrix (columns = body x, y, z axes in world coordinates).

    Shepperd's method: start from the largest of w, x, y, z, so we never divide by a tiny number.
    """
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(1.0 + tr)
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        q = [(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s]
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        q = [(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        q = [(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s]
    return quat_normalize(np.array(q))

"""Pose math helpers shared by runtime setup and diagnostics."""

from __future__ import annotations

import math

import numpy as np


def quat_wxyz_from_rpy_rad(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return a normalized wxyz quaternion from roll, pitch, yaw in radians."""
    cr = math.cos(0.5 * float(roll))
    sr = math.sin(0.5 * float(roll))
    cp = math.cos(0.5 * float(pitch))
    sp = math.sin(0.5 * float(pitch))
    cy = math.cos(0.5 * float(yaw))
    sy = math.sin(0.5 * float(yaw))
    quat = np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(quat))
    if norm <= 0.0 or not np.isfinite(norm):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return quat / norm


def rpy_rad_from_quat_wxyz(quat: np.ndarray) -> tuple[float, float, float]:
    """Return roll, pitch, yaw in radians from a wxyz quaternion."""
    q = np.asarray(quat, dtype=np.float64)
    if q.shape != (4,):
        return 0.0, 0.0, 0.0
    norm = float(np.linalg.norm(q))
    if norm <= 1.0e-12 or not np.isfinite(norm):
        return 0.0, 0.0, 0.0
    w, x, y, z = (float(v) for v in (q / norm))
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def angle_error_rad(a: float, b: float) -> float:
    """Return the absolute wrapped angular difference in radians."""
    return abs(math.atan2(math.sin(float(a) - float(b)), math.cos(float(a) - float(b))))

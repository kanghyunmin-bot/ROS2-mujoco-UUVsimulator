"""Quaternion helpers for physics contract audits."""

from __future__ import annotations

import math

import numpy as np


def normalized_quat_wxyz(quat: np.ndarray) -> tuple[float, float, float, float] | None:
    q = np.asarray(quat, dtype=np.float64)
    if q.shape != (4,):
        return None
    norm = float(np.linalg.norm(q))
    if norm <= 1.0e-12 or not np.isfinite(norm):
        return None
    return tuple(float(value) for value in (q / norm))


def rpy_rad_from_quat_wxyz(quat: np.ndarray) -> tuple[float, float, float]:
    normalized = normalized_quat_wxyz(quat)
    if normalized is None:
        return 0.0, 0.0, 0.0
    w, x, y, z = normalized
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


__all__ = ["normalized_quat_wxyz", "rpy_rad_from_quat_wxyz"]

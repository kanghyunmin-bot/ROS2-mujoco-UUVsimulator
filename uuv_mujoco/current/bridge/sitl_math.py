"""Small math helpers used by the SITL transport path."""

from __future__ import annotations

import numpy as np


def quat_to_rpy(quat: np.ndarray) -> tuple[float, float, float]:
    q = np.asarray(quat, dtype=np.float64)
    if q.shape[0] < 4:
        return 0.0, 0.0, 0.0
    w, x, y, z = q[:4]
    norm = float(np.linalg.norm(q[:4]))
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0
    q0, q1, q2, q3 = w / norm, x / norm, y / norm, z / norm
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1.0:
        pitch = np.copysign(np.pi / 2.0, sinp)
    else:
        pitch = np.arcsin(sinp)
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return float(roll), float(pitch), float(yaw)


def quat_to_rotmat(quat: np.ndarray) -> np.ndarray | None:
    q = np.asarray(quat, dtype=np.float64)
    if q.shape[0] < 4:
        return None
    norm = float(np.linalg.norm(q[:4]))
    if norm <= 1.0e-12:
        return None
    w, x, y, z = (q[:4] / norm).tolist()
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotmat_to_rpy(rot: np.ndarray) -> tuple[float, float, float]:
    r = np.asarray(rot, dtype=np.float64)
    if r.shape != (3, 3) or not np.all(np.isfinite(r)):
        return 0.0, 0.0, 0.0
    sy = float(np.clip(-r[2, 0], -1.0, 1.0))
    pitch = float(np.arcsin(sy))
    cos_pitch = float(np.cos(pitch))
    if abs(cos_pitch) > 1.0e-6:
        roll = float(np.arctan2(r[2, 1], r[2, 2]))
        yaw = float(np.arctan2(r[1, 0], r[0, 0]))
    else:
        roll = float(np.arctan2(-r[1, 2], r[1, 1]))
        yaw = 0.0
    return roll, pitch, yaw


__all__ = ["quat_to_rpy", "quat_to_rotmat", "rotmat_to_rpy"]

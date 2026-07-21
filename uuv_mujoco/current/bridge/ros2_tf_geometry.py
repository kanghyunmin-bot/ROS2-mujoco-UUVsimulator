"""Geometry helpers for ROS2 TF frame construction."""

from __future__ import annotations

from typing import Any

import numpy as np

from .ros2_math import rotmat_to_quat_wxyz


def quat_identity() -> np.ndarray:
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def quat_x_180() -> np.ndarray:
    return np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)


def camera_optical_quat() -> np.ndarray:
    rot_parent_child = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )
    return rotmat_to_quat_wxyz(rot_parent_child)


def site_local_pose(
    model: Any,
    site_id: int,
    *,
    fallback_pos: np.ndarray | None = None,
    fallback_quat: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    pos = np.zeros(3, dtype=np.float64) if fallback_pos is None else np.asarray(fallback_pos, dtype=np.float64)
    quat = quat_identity() if fallback_quat is None else np.asarray(fallback_quat, dtype=np.float64)
    if site_id >= 0:
        pos = np.asarray(model.site_pos[site_id], dtype=np.float64)
        quat = np.asarray(model.site_quat[site_id], dtype=np.float64)
    return pos, quat


__all__ = [
    "camera_optical_quat",
    "quat_identity",
    "quat_x_180",
    "site_local_pose",
]

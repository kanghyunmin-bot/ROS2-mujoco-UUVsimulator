"""Attitude extraction for real-start rows."""

from __future__ import annotations

import math

from real_start_common import finite
from real_start_geometry import quat_xyzw_to_rpy


def rpy_from_row(row: dict[str, str]) -> tuple[float, float, float, str]:
    sources = (
        ("local_pose", "local_pose_qx", "local_pose_qy", "local_pose_qz", "local_pose_qw"),
        ("imu_quat", "imu_quat_x", "imu_quat_y", "imu_quat_z", "imu_quat_w"),
    )
    for label, x_key, y_key, z_key, w_key in sources:
        x, y, z, w = (finite(row.get(key)) for key in (x_key, y_key, z_key, w_key))
        if not all(math.isfinite(v) for v in (x, y, z, w)):
            continue
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1.0e-9:
            continue
        roll, pitch, yaw = quat_xyzw_to_rpy(x / norm, y / norm, z / norm, w / norm)
        return roll, pitch, yaw, label
    return 0.0, 0.0, 0.0, "fallback_level"


__all__ = ["rpy_from_row"]

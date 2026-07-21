"""Local-pose velocity extraction for real-start rows."""

from __future__ import annotations

import math
import os

from real_start_common import finite
from real_start_geometry import mat_transpose_vec_mul, quat_xyzw_to_rotmat
from real_start_velocity_vectors import finite_row_quat_xyzw, finite_row_vector


def local_body_velocity_from_row(row: dict[str, str]) -> tuple[float, float, float, str] | None:
    local_vel = finite_row_vector(row, ("local_vel_x", "local_vel_y", "local_vel_z"))
    quat = finite_row_quat_xyzw(row)
    if local_vel is None or quat is None:
        return None
    rot_body_to_enu = quat_xyzw_to_rotmat(*quat)
    if rot_body_to_enu is None:
        return None
    body_vel = mat_transpose_vec_mul(rot_body_to_enu, local_vel)
    return float(body_vel[0]), float(body_vel[1]), float(body_vel[2]), "local_vel_rotated_to_body"


def local_xy_pose_z_fd_body_velocity_from_rows(
    rows: list[dict[str, str]],
    time_col: str,
    row: dict[str, str],
) -> tuple[float, float, float, str] | None:
    """Use local velocity x/y, but derive world-z velocity from pose finite difference."""
    local_vel = finite_row_vector(row, ("local_vel_x", "local_vel_y", "local_vel_z"))
    quat = finite_row_quat_xyzw(row)
    if local_vel is None or quat is None:
        return None
    rot_body_to_enu = quat_xyzw_to_rotmat(*quat)
    if rot_body_to_enu is None:
        return None
    source_t = finite(row.get(time_col))
    if not math.isfinite(source_t):
        return None
    window_s = pose_fd_window_from_env()
    pose_vz = pose_z_fd_velocity(rows, time_col, source_t, window_s)
    if pose_vz is None:
        return None
    local_with_pose_z = (float(local_vel[0]), float(local_vel[1]), float(pose_vz))
    body_vel = mat_transpose_vec_mul(rot_body_to_enu, local_with_pose_z)
    return (
        float(body_vel[0]),
        float(body_vel[1]),
        float(body_vel[2]),
        f"local_vel_xy_pose_z_fd_{window_s:.3f}s",
    )


def pose_fd_window_from_env() -> float:
    raw = os.environ.get("UUV_REAL_START_POSE_FD_WINDOW_S", "1.0")
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return 1.0
    return min(max(value, 0.05), 5.0)


def pose_z_fd_velocity(
    rows: list[dict[str, str]],
    time_col: str,
    source_t: float,
    window_s: float,
) -> float | None:
    end_t = float(source_t) + max(float(window_s), 0.05)
    candidates: list[tuple[float, float]] = []
    for candidate in rows:
        t = finite(candidate.get(time_col))
        z = finite(candidate.get("local_pose_z"))
        if math.isfinite(t) and math.isfinite(z) and float(source_t) <= t <= end_t:
            candidates.append((float(t), float(z)))
    if len(candidates) < 2:
        return None
    t0, z0 = candidates[0]
    t1, z1 = candidates[-1]
    dt = t1 - t0
    if dt <= 1.0e-6:
        return None
    return float((z1 - z0) / dt)


__all__ = [
    "local_body_velocity_from_row",
    "local_xy_pose_z_fd_body_velocity_from_rows",
    "pose_fd_window_from_env",
    "pose_z_fd_velocity",
]

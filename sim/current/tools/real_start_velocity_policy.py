"""Source-policy selection for real-start linear velocity."""

from __future__ import annotations

import os

from real_start_dvl_velocity import dvl_velocity_from_row
from real_start_local_velocity import local_body_velocity_from_row, local_xy_pose_z_fd_body_velocity_from_rows


def velocity_from_row(row: dict[str, str], source_policy: str = "local_pose") -> tuple[float, float, float, str]:
    source_policy = str(source_policy or "local_pose").strip().lower()
    if source_policy in {"dvl", "dvl_twist", "direct_sensor"}:
        dvl_vel = dvl_velocity_from_row(row, "dvl_twist")
        if dvl_vel is not None:
            return dvl_vel

    if source_policy in {"local_xy_dvl_z", "local_pose_xy_dvl_z", "hybrid_z"}:
        local_vel = local_body_velocity_from_row(row)
        dvl_vel = dvl_velocity_from_row(row, "dvl_twist")
        if local_vel is not None and dvl_vel is not None:
            return (
                float(local_vel[0]),
                float(local_vel[1]),
                float(dvl_vel[2]),
                "local_vel_xy_dvl_twist_z",
            )

    if source_policy in {"local_xy_dvl_z_blend", "local_pose_xy_dvl_z_blend", "hybrid_z_blend"}:
        local_vel = local_body_velocity_from_row(row)
        dvl_vel = dvl_velocity_from_row(row, "dvl_twist")
        if local_vel is not None and dvl_vel is not None:
            blend = dvl_z_blend_from_env()
            blended_z = (1.0 - blend) * float(local_vel[2]) + blend * float(dvl_vel[2])
            return (
                float(local_vel[0]),
                float(local_vel[1]),
                float(blended_z),
                f"local_vel_xy_dvl_twist_z_blend_{blend:.3f}",
            )

    local_vel = local_body_velocity_from_row(row)
    if local_vel is not None:
        return local_vel

    dvl_vel = dvl_velocity_from_row(row, "dvl_twist_fallback")
    if dvl_vel is not None:
        return dvl_vel
    return 0.0, 0.0, 0.0, "fallback_zero"


def velocity_from_rows(
    rows: list[dict[str, str]],
    time_col: str,
    row: dict[str, str],
    source_policy: str = "local_pose",
) -> tuple[float, float, float, str]:
    source_policy = str(source_policy or "local_pose").strip().lower()
    if source_policy in {"local_xy_pose_z_fd", "local_pose_xy_pose_z_fd", "pose_z_fd"}:
        pose_fd_vel = local_xy_pose_z_fd_body_velocity_from_rows(rows, time_col, row)
        if pose_fd_vel is not None:
            return pose_fd_vel
    return velocity_from_row(row, source_policy)


def dvl_z_blend_from_env() -> float:
    raw = os.environ.get("UUV_REAL_START_DVL_Z_BLEND", "1.0")
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return 1.0
    return min(max(value, 0.0), 1.0)


__all__ = ["dvl_z_blend_from_env", "velocity_from_row", "velocity_from_rows"]

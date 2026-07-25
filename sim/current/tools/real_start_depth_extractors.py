"""Depth and horizontal-position extraction for real-start rows."""

from __future__ import annotations

import math

from real_start_common import finite


def depth_from_row(row: dict[str, str]) -> tuple[float, str]:
    candidates = (
        ("depth_pose_depth_m", True),
        ("baro_depth_m", True),
        ("local_pose_z", False),
        ("static_pressure_delta_depth_m", True),
    )
    for key, positive_down in candidates:
        value = finite(row.get(key))
        if not math.isfinite(value):
            continue
        depth = value if positive_down else -value
        if math.isfinite(depth) and depth >= 0.0:
            return float(depth), key
    return 0.0, "fallback_zero"


def base_depth_from_row(row: dict[str, str]) -> tuple[float, str]:
    """Physical MuJoCo base_link depth from the real local pose contract."""
    local_z = finite(row.get("local_pose_z"))
    if math.isfinite(local_z):
        return float(max(0.0, -local_z)), "local_pose_z"
    depth_m, source = depth_from_row(row)
    return depth_m, source


def base_xy_from_row(row: dict[str, str]) -> tuple[float, float, str]:
    """Physical MuJoCo base_link horizontal position from the real local pose."""
    local_x = finite(row.get("local_pose_x"))
    local_y = finite(row.get("local_pose_y"))
    if math.isfinite(local_x) and math.isfinite(local_y):
        return float(local_x), float(local_y), "local_pose_xy"
    return 0.0, 0.0, "fallback_origin"


__all__ = ["base_depth_from_row", "base_xy_from_row", "depth_from_row"]

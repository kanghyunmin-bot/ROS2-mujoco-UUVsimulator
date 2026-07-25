"""Per-row Bar30 ground-pressure candidate extraction."""

from __future__ import annotations

import math

from real_start_common import finite


def baro_ground_pressure_candidates_from_row(
    row: dict[str, str],
    *,
    time_col: str,
    frontend_pa_per_m: float,
) -> tuple[float | None, float | None]:
    pressure_pa = finite(row.get("static_pressure_pa"))
    mode = str(row.get("mode", "")).strip().upper()
    t_s = finite(row.get(time_col))
    if not (math.isfinite(pressure_pa) and math.isfinite(t_s)):
        return None, None
    if mode and mode != "ALT_HOLD":
        return None, None

    depth_pose_candidate = None
    depth_pose_m = finite(row.get("depth_pose_depth_m"))
    if math.isfinite(depth_pose_m) and depth_pose_m >= 0.0:
        depth_pose_candidate = float(pressure_pa - frontend_pa_per_m * depth_pose_m)

    local_pose_candidate = None
    local_z = finite(row.get("local_pose_z"))
    if math.isfinite(local_z):
        local_depth_m = max(0.0, -local_z)
        local_pose_candidate = float(pressure_pa - frontend_pa_per_m * local_depth_m)

    return depth_pose_candidate, local_pose_candidate


__all__ = ["baro_ground_pressure_candidates_from_row"]

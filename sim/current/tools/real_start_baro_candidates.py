"""Candidate collection for real-start Bar30/AP_Baro pressure datum inference."""

from __future__ import annotations

import statistics

from real_start_baro_row import baro_ground_pressure_candidates_from_row


def collect_baro_ground_pressure_candidates(
    rows: list[dict[str, str]],
    *,
    time_col: str,
    frontend_pa_per_m: float,
) -> dict[str, list[float]]:
    depth_pose_candidates: list[float] = []
    local_pose_candidates: list[float] = []
    for row in rows:
        depth_pose_candidate, local_pose_candidate = baro_ground_pressure_candidates_from_row(
            row,
            time_col=time_col,
            frontend_pa_per_m=frontend_pa_per_m,
        )
        if depth_pose_candidate is not None:
            depth_pose_candidates.append(depth_pose_candidate)
        if local_pose_candidate is not None:
            local_pose_candidates.append(local_pose_candidate)
    return {
        "depth_pose": depth_pose_candidates,
        "local_pose": local_pose_candidates,
    }


def select_baro_ground_pressure_candidate(candidates: dict[str, list[float]]) -> tuple[float, str]:
    depth_pose_candidates = candidates.get("depth_pose") or []
    if depth_pose_candidates:
        return (
            float(statistics.median(depth_pose_candidates)),
            "static_pressure_minus_depth_pose_depth_m_fit",
        )

    local_pose_candidates = candidates.get("local_pose") or []
    if local_pose_candidates:
        return (
            float(statistics.median(local_pose_candidates)),
            "static_pressure_minus_local_pose_z_fit",
        )

    return float("nan"), "unavailable"


__all__ = ["collect_baro_ground_pressure_candidates", "select_baro_ground_pressure_candidate"]

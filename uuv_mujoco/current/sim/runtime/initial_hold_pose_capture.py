"""Capture helpers for initial hold pose/depth state."""

from __future__ import annotations

from typing import Any

import numpy as np


def coerce_body_vector(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    return np.asarray(value, dtype=np.float64)


def capture_initial_hold_pose(
    *,
    data,
    world_qpos_adr: int,
    water_surface_z: float,
    depth_m: float | None = None,
) -> np.ndarray:
    """Capture the current free-joint pose for repeated hold enforcement."""
    pose_qpos = data.qpos[world_qpos_adr : world_qpos_adr + 7].copy()
    if depth_m is not None:
        pose_qpos[2] = water_surface_z - float(depth_m)
    return pose_qpos


__all__ = ["capture_initial_hold_pose", "coerce_body_vector"]

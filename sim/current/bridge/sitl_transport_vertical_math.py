"""Math helpers for SitlTransport legacy base-link vertical fallback."""

from __future__ import annotations

import numpy as np


def finite_position_velocity(position: np.ndarray, velocity: np.ndarray) -> bool:
    return bool(np.all(np.isfinite(position)) and np.all(np.isfinite(velocity)))


def positive_down_depth(water_surface_z: float, pos_z_enu: float) -> float:
    return float(max(0.0, float(water_surface_z) - float(pos_z_enu)))


def ned_position_velocity(
    *,
    enu_to_ned: np.ndarray,
    base_pos_enu: np.ndarray,
    base_vel_enu: np.ndarray,
    depth_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    pos_ned = enu_to_ned @ np.asarray(base_pos_enu, dtype=np.float64)
    vel_ned = enu_to_ned @ np.asarray(base_vel_enu, dtype=np.float64)
    pos_ned[2] = float(depth_m)
    vel_ned[2] = float(-float(base_vel_enu[2]))
    return pos_ned, vel_ned


__all__ = ["finite_position_velocity", "ned_position_velocity", "positive_down_depth"]

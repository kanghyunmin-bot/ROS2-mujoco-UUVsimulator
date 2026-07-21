"""Vertical velocity fallback policy for ArduSub SITL Bar30 state."""

from __future__ import annotations

import numpy as np


def estimate_sitl_vertical_velocity_d(
    self,
    *,
    bar30_vel_enu: np.ndarray | None,
    physical_depth_m: float,
    base_vel_enu: np.ndarray,
    sim_t: float,
) -> float:
    vel_d = float("nan")
    if bar30_vel_enu is not None and np.all(np.isfinite(bar30_vel_enu)):
        vel_d = float(-float(bar30_vel_enu[2]))

    prev_depth = self._sitl_bar30_prev_depth_m
    prev_t = self._sitl_bar30_prev_t
    if prev_depth is not None and prev_t is not None:
        dt = float(sim_t) - float(prev_t)
        if 1.0e-4 <= dt <= 0.2 and (not np.isfinite(vel_d)):
            candidate = (float(physical_depth_m) - float(prev_depth)) / dt
            if np.isfinite(candidate):
                vel_d = float(np.clip(candidate, -5.0, 5.0))
    self._sitl_bar30_prev_depth_m = float(physical_depth_m)
    self._sitl_bar30_prev_t = float(sim_t)

    if not np.isfinite(vel_d):
        vel_d = float(-float(base_vel_enu[2]))
    return vel_d


__all__ = ["estimate_sitl_vertical_velocity_d"]

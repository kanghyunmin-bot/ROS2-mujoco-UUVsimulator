"""Vertical-state helpers for SitlTransport MuJoCo truth fallback."""

from __future__ import annotations

import numpy as np

from bridge.sitl_replay import pressure_abs_from_depth_m as pressure_abs_from_depth
from bridge.sitl_types import VerticalEstimate
from bridge.sitl_transport_vertical_math import finite_position_velocity, ned_position_velocity, positive_down_depth


def pressure_abs_from_depth_m(self, depth_m: float) -> float:
    return pressure_abs_from_depth(
        depth_m,
        self._bar30_surface_pressure_pa,
        self._bar30_water_density,
        self._bar30_gravity,
    )


def estimate_base_velocity_enu(self, sim_t: float, base_pos_enu: np.ndarray) -> np.ndarray:
    """Return simple finite-difference base velocity in ENU."""
    vel_enu = np.zeros(3, dtype=np.float64)
    prev_t = self._sitl_prev_sim_t
    prev_pos = self._sitl_prev_pos_enu
    if prev_t is not None and prev_pos is not None:
        dt = sim_t - float(prev_t)
        if 1.0e-4 <= dt <= 0.2:
            vel_fd = (base_pos_enu - prev_pos) / dt
            if np.all(np.isfinite(vel_fd)):
                vel_enu = np.clip(vel_fd, -8.0, 8.0)
    self._sitl_prev_sim_t = sim_t
    self._sitl_prev_pos_enu = base_pos_enu.copy()
    return vel_enu


def estimate_vertical_state(
    self,
    base_pos_enu: np.ndarray,
    base_vel_enu: np.ndarray,
) -> VerticalEstimate | None:
    """Legacy base-link vertical fallback."""
    if base_pos_enu is None or base_vel_enu is None:
        return None
    if not finite_position_velocity(base_pos_enu, base_vel_enu):
        return None

    base_depth_m = positive_down_depth(self._water_surface_z, base_pos_enu[2])
    pos_ned, vel_ned = ned_position_velocity(
        enu_to_ned=self._enu_to_ned,
        base_pos_enu=base_pos_enu,
        base_vel_enu=base_vel_enu,
        depth_m=base_depth_m,
    )
    pressure_pa = self.pressure_abs_from_depth_m(base_depth_m)
    alt_m = float(self._sitl_home_alt_m - base_depth_m)
    return VerticalEstimate(
        depth_m=base_depth_m,
        pressure_pa=pressure_pa,
        pos_ned=pos_ned,
        vel_ned=vel_ned,
        alt_m=alt_m,
    )


__all__ = ["estimate_base_velocity_enu", "estimate_vertical_state", "pressure_abs_from_depth_m"]

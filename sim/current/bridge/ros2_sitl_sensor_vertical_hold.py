"""Initial-hold helpers for SITL Bar30 vertical sensor state."""

from __future__ import annotations

import numpy as np

from .ros2_sitl_sensor_types import BaseKinematicState
from .sitl_types import VerticalEstimate


def bar30_velocity_after_initial_hold(
    self,
    *,
    bar30_vel_enu: np.ndarray,
    bar30_pos_enu: np.ndarray,
    base: BaseKinematicState,
) -> np.ndarray:
    if not base.zero_vertical_reason:
        return bar30_vel_enu

    held_vel_enu = np.asarray(bar30_vel_enu, dtype=np.float64).copy()
    if self._sitl_initial_depth_hold_active:
        held_vel_enu[:] = 0.0
    else:
        held_vel_enu[2] = 0.0
    self._sitl_bar30_prev_depth_m = float(max(0.0, self._water_surface_z - float(bar30_pos_enu[2])))
    self._sitl_bar30_prev_t = base.sim_t
    return held_vel_enu


def vertical_estimate_after_initial_hold(
    *,
    base: BaseKinematicState,
    vertical_estimate: VerticalEstimate,
) -> VerticalEstimate:
    if not base.zero_vertical_reason:
        return vertical_estimate

    sitl_vel_ned = vertical_estimate.vel_ned.copy()
    sitl_vel_ned[2] = 0.0
    return VerticalEstimate(
        depth_m=vertical_estimate.depth_m,
        pressure_pa=vertical_estimate.pressure_pa,
        pos_ned=vertical_estimate.pos_ned.copy(),
        vel_ned=sitl_vel_ned,
        alt_m=vertical_estimate.alt_m,
        extnav_pos_ned=(
            None
            if vertical_estimate.extnav_pos_ned is None
            else vertical_estimate.extnav_pos_ned.copy()
        ),
    )


__all__ = ["bar30_velocity_after_initial_hold", "vertical_estimate_after_initial_hold"]

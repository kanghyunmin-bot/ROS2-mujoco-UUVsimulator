"""Vertical truth and Bar30 pressure helpers for Ros2Bridge."""

from __future__ import annotations

import numpy as np

from .ros2_math import pressure_abs_from_depth_m
from .sitl_types import VerticalEstimate


def _estimate_base_accel_enu(self, sim_t: float, base_vel_enu: np.ndarray) -> np.ndarray:
    accel_enu = np.zeros(3, dtype=np.float64)
    prev_t = self._sitl_prev_vel_sim_t
    prev_vel = self._sitl_prev_vel_enu
    if prev_t is not None and prev_vel is not None:
        dt = sim_t - float(prev_t)
        if 1.0e-4 <= dt <= 0.2:
            accel_fd = (base_vel_enu - prev_vel) / dt
            if np.all(np.isfinite(accel_fd)):
                accel_enu = np.clip(accel_fd, -self._imu_acc_clip_mps2, self._imu_acc_clip_mps2)
    self._sitl_prev_vel_sim_t = sim_t
    self._sitl_prev_vel_enu = base_vel_enu.copy()
    return accel_enu


def _estimate_vertical_truth(self, base_pos_enu: np.ndarray, base_vel_enu: np.ndarray) -> VerticalEstimate:
    depth_m = float(max(0.0, self._water_surface_z - float(base_pos_enu[2])))
    pos_ned = self._enu_to_ned @ base_pos_enu
    vel_ned = self._enu_to_ned @ base_vel_enu
    pos_ned[2] = depth_m
    vel_ned[2] = -base_vel_enu[2]
    pressure_pa = pressure_abs_from_depth_m(
        depth_m,
        self._bar30_surface_pressure_pa,
        self._bar30_water_density,
        self._bar30_gravity,
    )
    return VerticalEstimate(
        depth_m=depth_m,
        pressure_pa=pressure_pa,
        pos_ned=pos_ned,
        vel_ned=vel_ned,
        alt_m=self._sitl_home_alt_m - depth_m,
        extnav_pos_ned=pos_ned.copy(),
    )


def _estimate_bar30_pressure_pa(
    self,
    bar30_pos_enu: np.ndarray,
    depth_bias_m: float = 0.0,
) -> float:
    bar30_depth_m = float(max(0.0, self._water_surface_z - float(bar30_pos_enu[2]) + float(depth_bias_m)))
    return pressure_abs_from_depth_m(
        bar30_depth_m,
        self._bar30_surface_pressure_pa,
        self._bar30_water_density,
        self._bar30_gravity,
    )


__all__ = [
    "_estimate_bar30_pressure_pa",
    "_estimate_base_accel_enu",
    "_estimate_vertical_truth",
]

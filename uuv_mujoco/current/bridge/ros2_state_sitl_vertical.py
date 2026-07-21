"""Canonical ArduSub SITL vertical estimate helper for Ros2Bridge."""

from __future__ import annotations

import numpy as np

from .ros2_state_sitl_baro import (
    sitl_bar30_physical_depth_m,
    sitl_bar30_pressure_pa,
    sitl_contract_depth_m,
)
from .ros2_state_sitl_frames import sitl_vertical_frames
from .ros2_state_sitl_velocity import estimate_sitl_vertical_velocity_d
from .sitl_types import VerticalEstimate


def _estimate_sitl_vertical(
    self,
    base_pos_enu: np.ndarray,
    base_vel_enu: np.ndarray,
    bar30_pos_enu: np.ndarray,
    bar30_vel_enu: np.ndarray | None,
    sim_t: float,
) -> VerticalEstimate:
    """Canonical vertical state for ArduSub SITL."""
    physical_depth_m = sitl_bar30_physical_depth_m(
        water_surface_z=self._water_surface_z,
        bar30_pos_enu=bar30_pos_enu,
        depth_sensor_bias_m=self._sitl_depth_sensor_bias_m,
    )
    pressure_pa = sitl_bar30_pressure_pa(
        physical_depth_m=physical_depth_m,
        surface_pressure_pa=self._bar30_surface_pressure_pa,
        water_density=self._bar30_water_density,
        gravity=self._bar30_gravity,
    )
    depth_m = sitl_contract_depth_m(self, physical_depth_m=physical_depth_m, pressure_pa=pressure_pa)
    vel_d = estimate_sitl_vertical_velocity_d(
        self,
        bar30_vel_enu=bar30_vel_enu,
        physical_depth_m=physical_depth_m,
        base_vel_enu=base_vel_enu,
        sim_t=sim_t,
    )
    pos_ned, vel_ned, extnav_pos_ned = sitl_vertical_frames(
        self,
        base_pos_enu=base_pos_enu,
        base_vel_enu=base_vel_enu,
        depth_m=depth_m,
        vel_d=vel_d,
    )
    alt_m = float(self._sitl_home_alt_m - depth_m)
    return VerticalEstimate(
        depth_m=depth_m,
        pressure_pa=pressure_pa,
        pos_ned=pos_ned,
        vel_ned=vel_ned,
        alt_m=alt_m,
        extnav_pos_ned=extnav_pos_ned,
    )


__all__ = ["_estimate_sitl_vertical"]

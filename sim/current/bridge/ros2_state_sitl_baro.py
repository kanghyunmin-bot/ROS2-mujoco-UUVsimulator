"""Bar30 pressure/depth helpers for ArduSub SITL vertical state."""

from __future__ import annotations

import numpy as np

from .ros2_math import pressure_abs_from_depth_m


def sitl_bar30_physical_depth_m(
    *,
    water_surface_z: float,
    bar30_pos_enu: np.ndarray,
    depth_sensor_bias_m: float,
) -> float:
    raw_bar30_depth_m = float(water_surface_z) - float(bar30_pos_enu[2])
    bar30_abs_depth_m = float(max(0.0, raw_bar30_depth_m))
    return float(max(0.0, bar30_abs_depth_m + float(depth_sensor_bias_m)))


def sitl_bar30_pressure_pa(
    *,
    physical_depth_m: float,
    surface_pressure_pa: float,
    water_density: float,
    gravity: float,
) -> float:
    return pressure_abs_from_depth_m(
        physical_depth_m,
        surface_pressure_pa,
        water_density,
        gravity,
    )


def sitl_contract_depth_m(self, *, physical_depth_m: float, pressure_pa: float) -> float:
    if self._sitl_baro_depth_contract == "frontend_match":
        return float(self._baro_pressure_law.sitl_depth_m_for_frontend_match(pressure_pa))
    return float(physical_depth_m)


__all__ = [
    "sitl_bar30_physical_depth_m",
    "sitl_bar30_pressure_pa",
    "sitl_contract_depth_m",
]

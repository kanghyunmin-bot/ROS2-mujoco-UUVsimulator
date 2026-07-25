"""Compatibility exports for shared ArduSub SITL contracts.

The canonical implementation lives under `sim.contracts`.  This module remains
so existing bridge imports keep working while the runtime is migrated one caller
at a time.
"""

from __future__ import annotations

from sim.contracts.baro import (
    AP_BARO_FRONTEND_PA_PER_M,
    AP_BARO_SITL_SSL_AIR_PRESSURE_PA,
    AP_BARO_SITL_WATER_DENSITY_KG_M3,
    STANDARD_GRAVITY_M_S2,
    BaroPressureLaw,
    surface_pressure_for_depth_sample,
)
from sim.contracts.rates import REAL_ROBOT_SENSOR_RATES_HZ

__all__ = [
    "AP_BARO_FRONTEND_PA_PER_M",
    "AP_BARO_SITL_SSL_AIR_PRESSURE_PA",
    "AP_BARO_SITL_WATER_DENSITY_KG_M3",
    "STANDARD_GRAVITY_M_S2",
    "REAL_ROBOT_SENSOR_RATES_HZ",
    "BaroPressureLaw",
    "surface_pressure_for_depth_sample",
]

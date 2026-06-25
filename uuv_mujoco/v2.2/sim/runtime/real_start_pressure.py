"""Pressure helpers for real-start runtime measurements."""

from __future__ import annotations

import math

from .real_start_types import EnvFloat


def pressure_now_pa(
    *,
    env_float: EnvFloat,
    model_density: float,
    bar30_depth_m: float,
) -> float:
    pressure_surface_pa = env_float("ROS2_UUV_BAR30_SURFACE_PRESSURE_PA", math.nan)
    pressure_rho = env_float("ROS2_UUV_BAR30_WATER_DENSITY", float(model_density))
    pressure_gravity = env_float("ROS2_UUV_BAR30_GRAVITY", 9.80665)
    if all(math.isfinite(v) for v in (pressure_surface_pa, pressure_rho, pressure_gravity, bar30_depth_m)):
        return float(pressure_surface_pa + pressure_rho * pressure_gravity * max(0.0, bar30_depth_m))
    return math.nan


__all__ = ["pressure_now_pa"]

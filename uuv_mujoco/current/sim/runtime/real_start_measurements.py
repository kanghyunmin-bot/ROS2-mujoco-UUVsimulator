"""Current-state error calculations for real-start status payloads."""

from __future__ import annotations

import math

import numpy as np

from .real_start_measurement_errors import (
    attitude_error_or_nan,
    depth_now_for_contract,
    scalar_error_or_nan,
    vector_error_or_inf,
    xy_error_or_nan,
)
from .real_start_pressure import pressure_now_pa
from .real_start_types import EnvFloat, RealStartMeasurements, RealStartTargets


def compute_real_start_measurements(
    *,
    env_float: EnvFloat,
    targets: RealStartTargets,
    base_depth_m: float,
    bar30_depth_m: float,
    base_xy_m: np.ndarray,
    current_rpy_rad: tuple[float, float, float],
    release_linear_velocity_body,
    release_angular_velocity_body,
    model_density: float,
) -> RealStartMeasurements:
    base_xy_now = np.asarray(base_xy_m, dtype=np.float64)
    xy_error = xy_error_or_nan(base_xy_now, target_x=targets.target_x, target_y=targets.target_y)
    depth_now = depth_now_for_contract(
        depth_contract=targets.depth_contract,
        base_depth_m=base_depth_m,
        bar30_depth_m=bar30_depth_m,
    )
    depth_error = scalar_error_or_nan(depth_now, targets.target_depth)
    pressure_now_value_pa = pressure_now_pa(
        env_float=env_float,
        model_density=model_density,
        bar30_depth_m=bar30_depth_m,
    )
    pressure_error_pa = scalar_error_or_nan(pressure_now_value_pa, targets.target_pressure_pa)
    attitude_error = attitude_error_or_nan(current_rpy_rad, targets.target_rpy)

    return RealStartMeasurements(
        depth_now=float(depth_now),
        depth_error=float(depth_error) if math.isfinite(depth_error) else math.nan,
        base_xy_now=base_xy_now,
        xy_error=float(xy_error) if math.isfinite(xy_error) else math.nan,
        pressure_now_pa=float(pressure_now_value_pa) if math.isfinite(pressure_now_value_pa) else math.nan,
        pressure_error_pa=float(pressure_error_pa) if math.isfinite(pressure_error_pa) else math.nan,
        attitude_error=float(attitude_error) if math.isfinite(attitude_error) else math.nan,
        velocity_error=vector_error_or_inf(release_linear_velocity_body, targets.target_v),
        angular_velocity_error=vector_error_or_inf(release_angular_velocity_body, targets.target_w),
    )


__all__ = ["compute_real_start_measurements"]

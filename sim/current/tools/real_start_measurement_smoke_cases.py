"""Smoke cases for real-start measurement calculations."""

from __future__ import annotations

import math

import numpy as np

from sim.runtime.real_start_measurements import compute_real_start_measurements
from sim.runtime.real_start_types import RealStartTargets


def env_float(name: str, default: float) -> float:
    values = {
        "ROS2_UUV_BAR30_SURFACE_PRESSURE_PA": 101000.0,
        "ROS2_UUV_BAR30_WATER_DENSITY": 1000.0,
        "ROS2_UUV_BAR30_GRAVITY": 10.0,
    }
    return float(values.get(name, default))


def assert_close(actual: float, expected: float, label: str) -> None:
    if abs(float(actual) - float(expected)) > 1.0e-9:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


def check_bar30_depth_contract() -> None:
    targets = RealStartTargets(
        target_depth=2.0,
        depth_contract="bar30",
        target_rpy=(0.0, 0.0, -math.pi + 0.1),
        target_x=1.0,
        target_y=2.0,
        target_pressure_pa=121000.0,
        target_v=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        target_w=np.array([0.0, 0.0, 0.0], dtype=np.float64),
        source_t_s=0.0,
        pressure_tol_pa=1.0,
        xy_tol_m=0.1,
    )
    measurements = compute_real_start_measurements(
        env_float=env_float,
        targets=targets,
        base_depth_m=9.0,
        bar30_depth_m=2.0,
        base_xy_m=np.array([4.0, 6.0], dtype=np.float64),
        current_rpy_rad=(0.0, 0.0, math.pi - 0.1),
        release_linear_velocity_body=np.array([1.0, 2.0, 0.0], dtype=np.float64),
        release_angular_velocity_body=None,
        model_density=997.0,
    )
    assert_close(measurements.depth_now, 2.0, "bar30 depth contract")
    assert_close(measurements.depth_error, 0.0, "depth error")
    assert_close(measurements.xy_error, 5.0, "xy error")
    assert_close(measurements.pressure_now_pa, 121000.0, "pressure now")
    assert_close(measurements.pressure_error_pa, 0.0, "pressure error")
    assert_close(measurements.attitude_error, 0.2, "wrapped yaw attitude error")
    assert_close(measurements.velocity_error, 2.0, "linear velocity error")
    if not math.isinf(measurements.angular_velocity_error):
        raise AssertionError("missing angular velocity should be inf")


def check_base_link_depth_contract() -> None:
    measurements = compute_real_start_measurements(
        env_float=env_float,
        targets=RealStartTargets(
            target_depth=9.0,
            depth_contract="base_link",
            target_rpy=(math.nan, math.nan, math.nan),
            target_x=math.nan,
            target_y=math.nan,
            target_pressure_pa=math.nan,
            target_v=np.zeros(3, dtype=np.float64),
            target_w=np.zeros(3, dtype=np.float64),
            source_t_s=0.0,
            pressure_tol_pa=1.0,
            xy_tol_m=0.1,
        ),
        base_depth_m=9.0,
        bar30_depth_m=2.0,
        base_xy_m=np.array([0.0, 0.0], dtype=np.float64),
        current_rpy_rad=(1.0, 2.0, 3.0),
        release_linear_velocity_body=np.zeros(3, dtype=np.float64),
        release_angular_velocity_body=np.zeros(3, dtype=np.float64),
        model_density=997.0,
    )
    assert_close(measurements.depth_now, 9.0, "base-link depth contract")
    if not math.isnan(measurements.xy_error):
        raise AssertionError("missing XY target should produce NaN")


__all__ = ["check_bar30_depth_contract", "check_base_link_depth_contract"]

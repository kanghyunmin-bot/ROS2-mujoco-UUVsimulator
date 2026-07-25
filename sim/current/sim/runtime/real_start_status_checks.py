"""Individual real-start status mismatch checks."""

from __future__ import annotations

from .real_start_status_predicates import (
    all_finite,
    finite_abs_exceeds,
    finite_and_exceeds,
    missing_or_abs_exceeds,
    missing_or_exceeds,
)
from .real_start_types import RealStartMeasurements, RealStartTargets


def missing_real_start_contract(targets: RealStartTargets) -> bool:
    return missing_or_exceeds(targets.target_depth, float("inf")) or not all_finite(targets.target_rpy)


def depth_mismatch(measurements: RealStartMeasurements, tolerance_m: float) -> bool:
    return missing_or_abs_exceeds(measurements.depth_error, tolerance_m)


def pressure_mismatch(targets: RealStartTargets, measurements: RealStartMeasurements) -> bool:
    return finite_abs_exceeds(measurements.pressure_error_pa, targets.pressure_tol_pa)


def xy_mismatch(targets: RealStartTargets, measurements: RealStartMeasurements) -> bool:
    return finite_and_exceeds(measurements.xy_error, targets.xy_tol_m)


def attitude_mismatch(measurements: RealStartMeasurements, tolerance_rad: float) -> bool:
    return missing_or_exceeds(measurements.attitude_error, tolerance_rad)


def velocity_mismatch(measurements: RealStartMeasurements, tolerance_mps: float) -> bool:
    return missing_or_exceeds(measurements.velocity_error, tolerance_mps)


def angular_velocity_mismatch(
    targets: RealStartTargets,
    measurements: RealStartMeasurements,
    tolerance_mps: float,
) -> bool:
    return bool(
        all_finite(targets.target_w)
        and missing_or_exceeds(measurements.angular_velocity_error, tolerance_mps)
    )


__all__ = [
    "angular_velocity_mismatch",
    "attitude_mismatch",
    "depth_mismatch",
    "missing_real_start_contract",
    "pressure_mismatch",
    "velocity_mismatch",
    "xy_mismatch",
]

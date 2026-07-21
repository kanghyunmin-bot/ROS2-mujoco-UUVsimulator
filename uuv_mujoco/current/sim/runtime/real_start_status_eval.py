"""Real-start ready/mismatch status evaluation."""

from __future__ import annotations

from .real_start_types import RealStartMeasurements, RealStartTargets
from .real_start_status_checks import (
    angular_velocity_mismatch,
    attitude_mismatch,
    depth_mismatch,
    missing_real_start_contract,
    pressure_mismatch,
    velocity_mismatch,
    xy_mismatch,
)


def determine_real_start_status(
    *,
    targets: RealStartTargets,
    measurements: RealStartMeasurements,
    depth_tolerance_m: float,
    attitude_tolerance_rad: float,
    velocity_tolerance_mps: float,
) -> tuple[bool, str]:
    if missing_real_start_contract(targets):
        return False, "missing_contract"
    if depth_mismatch(measurements, depth_tolerance_m):
        return False, "depth_mismatch"
    if pressure_mismatch(targets, measurements):
        return False, "pressure_mismatch"
    if xy_mismatch(targets, measurements):
        return False, "xy_mismatch"
    if attitude_mismatch(measurements, attitude_tolerance_rad):
        return False, "attitude_mismatch"
    if velocity_mismatch(measurements, velocity_tolerance_mps):
        return False, "velocity_mismatch"
    if angular_velocity_mismatch(targets, measurements, velocity_tolerance_mps):
        return False, "angular_velocity_mismatch"
    return True, "ok"


__all__ = ["determine_real_start_status"]

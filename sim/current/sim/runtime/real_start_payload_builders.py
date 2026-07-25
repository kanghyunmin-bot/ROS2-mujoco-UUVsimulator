"""Payload builders for real-start status publication."""

from __future__ import annotations

from .real_start_payload_values import finite_float_or_nan
from .real_start_types import RealStartMeasurements, RealStartTargets


def real_start_not_required_payload(
    *,
    hold_active: bool,
    latched_payload: dict[str, object] | None,
) -> tuple[dict[str, object], dict[str, object] | None]:
    return (
        {
            "required": False,
            "ok": True,
            "released": not bool(hold_active),
            "hold_active": bool(hold_active),
            "status": "not_required",
        },
        latched_payload,
    )


def build_required_payload(
    *,
    targets: RealStartTargets,
    measurements: RealStartMeasurements,
    ok: bool,
    status: str,
    hold_active: bool,
    base_depth_m: float,
    bar30_depth_m: float,
    depth_tolerance_m: float,
    attitude_tolerance_rad: float,
    velocity_tolerance_mps: float,
) -> dict[str, object]:
    return {
        "required": True,
        "ok": bool(ok),
        "released": not bool(hold_active),
        "hold_active": bool(hold_active),
        "status": status,
        "source_t_s": targets.source_t_s,
        "depth_m": float(measurements.depth_now),
        "target_depth_m": finite_float_or_nan(targets.target_depth),
        "depth_error_m": finite_float_or_nan(measurements.depth_error),
        "depth_contract": targets.depth_contract,
        "base_depth_m": float(base_depth_m),
        "bar30_depth_m": float(bar30_depth_m),
        "base_x_m": float(measurements.base_xy_now[0]),
        "base_y_m": float(measurements.base_xy_now[1]),
        "target_base_x_m": finite_float_or_nan(targets.target_x),
        "target_base_y_m": finite_float_or_nan(targets.target_y),
        "xy_error_m": finite_float_or_nan(measurements.xy_error),
        "pressure_pa": finite_float_or_nan(measurements.pressure_now_pa),
        "target_pressure_pa": finite_float_or_nan(targets.target_pressure_pa),
        "pressure_error_pa": finite_float_or_nan(measurements.pressure_error_pa),
        "attitude_error_rad": finite_float_or_nan(measurements.attitude_error),
        "velocity_error_mps": finite_float_or_nan(measurements.velocity_error),
        "angular_velocity_error_radps": finite_float_or_nan(measurements.angular_velocity_error),
        "depth_tolerance_m": depth_tolerance_m,
        "xy_tolerance_m": targets.xy_tol_m,
        "pressure_tolerance_pa": targets.pressure_tol_pa,
        "attitude_tolerance_rad": attitude_tolerance_rad,
        "velocity_tolerance_mps": velocity_tolerance_mps,
    }


def build_latched_payload(
    *,
    latched_payload: dict[str, object],
    hold_active: bool,
    measurements: RealStartMeasurements,
) -> dict[str, object]:
    latched = dict(latched_payload)
    latched.update(
        {
            "ok": True,
            "released": not bool(hold_active),
            "hold_active": bool(hold_active),
            "status": "released" if not bool(hold_active) else "ok",
            "current_depth_m": float(measurements.depth_now),
            "current_depth_error_m": finite_float_or_nan(measurements.depth_error),
            "current_pressure_error_pa": finite_float_or_nan(measurements.pressure_error_pa),
            "current_attitude_error_rad": finite_float_or_nan(measurements.attitude_error),
        }
    )
    return latched


__all__ = [
    "build_latched_payload",
    "build_required_payload",
    "real_start_not_required_payload",
]

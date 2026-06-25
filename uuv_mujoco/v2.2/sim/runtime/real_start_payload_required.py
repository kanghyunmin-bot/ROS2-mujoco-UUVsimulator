"""Required real-start payload assembly helpers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .real_start_measurements import compute_real_start_measurements
from .real_start_payload_status import (
    build_latched_payload,
    build_required_payload,
    determine_real_start_status,
)
from .real_start_targets import load_real_start_targets
from .real_start_types import EnvFloat, RealStartMeasurements


@dataclass(frozen=True)
class RequiredRealStartInputs:
    env_float: EnvFloat
    hold_active: bool
    base_depth_m: float
    bar30_depth_m: float
    base_xy_m: np.ndarray
    current_rpy_rad: tuple[float, float, float]
    release_linear_velocity_body: object
    release_angular_velocity_body: object
    model_density: float
    depth_tolerance_m: float
    attitude_tolerance_rad: float
    velocity_tolerance_mps: float


@dataclass(frozen=True)
class RequiredRealStartAttempt:
    payload: dict[str, object]
    measurements: RealStartMeasurements
    ok: bool


def build_required_real_start_attempt(inputs: RequiredRealStartInputs) -> RequiredRealStartAttempt:
    """Evaluate current state against the real-start target contract."""

    targets = load_real_start_targets(inputs.env_float)
    measurements = compute_real_start_measurements(
        env_float=inputs.env_float,
        targets=targets,
        base_depth_m=inputs.base_depth_m,
        bar30_depth_m=inputs.bar30_depth_m,
        base_xy_m=inputs.base_xy_m,
        current_rpy_rad=inputs.current_rpy_rad,
        release_linear_velocity_body=inputs.release_linear_velocity_body,
        release_angular_velocity_body=inputs.release_angular_velocity_body,
        model_density=inputs.model_density,
    )
    ok, status = determine_real_start_status(
        targets=targets,
        measurements=measurements,
        depth_tolerance_m=inputs.depth_tolerance_m,
        attitude_tolerance_rad=inputs.attitude_tolerance_rad,
        velocity_tolerance_mps=inputs.velocity_tolerance_mps,
    )
    payload = build_required_payload(
        targets=targets,
        measurements=measurements,
        ok=ok,
        status=status,
        hold_active=inputs.hold_active,
        base_depth_m=inputs.base_depth_m,
        bar30_depth_m=inputs.bar30_depth_m,
        depth_tolerance_m=inputs.depth_tolerance_m,
        attitude_tolerance_rad=inputs.attitude_tolerance_rad,
        velocity_tolerance_mps=inputs.velocity_tolerance_mps,
    )
    return RequiredRealStartAttempt(payload=payload, measurements=measurements, ok=ok)


def resolve_required_real_start_payload(
    *,
    attempt: RequiredRealStartAttempt,
    latched_payload: dict[str, object] | None,
    hold_active: bool,
) -> tuple[dict[str, object], dict[str, object] | None]:
    """Return the payload/latch pair after required-start evaluation."""

    if attempt.ok:
        return attempt.payload, dict(attempt.payload)
    if latched_payload is not None:
        return (
            build_latched_payload(
                latched_payload=latched_payload,
                hold_active=hold_active,
                measurements=attempt.measurements,
            ),
            latched_payload,
        )
    return attempt.payload, latched_payload


__all__ = [
    "RequiredRealStartAttempt",
    "RequiredRealStartInputs",
    "build_required_real_start_attempt",
    "resolve_required_real_start_payload",
]

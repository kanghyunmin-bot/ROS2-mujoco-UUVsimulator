"""Public real-start status payload builder."""

from __future__ import annotations

import numpy as np

from .real_start_payload_required import (
    RequiredRealStartInputs,
    build_required_real_start_attempt,
    resolve_required_real_start_payload,
)
from .real_start_payload_status import (
    real_start_not_required_payload,
)
from .real_start_types import EnvFloat


def build_real_start_status_payload(
    *,
    env_float: EnvFloat,
    required: bool,
    hold_active: bool,
    latched_payload: dict[str, object] | None,
    base_depth_m: float,
    bar30_depth_m: float,
    base_xy_m: np.ndarray,
    current_rpy_rad: tuple[float, float, float],
    release_linear_velocity_body,
    release_angular_velocity_body,
    model_density: float,
    depth_tolerance_m: float,
    attitude_tolerance_rad: float,
    velocity_tolerance_mps: float,
) -> tuple[dict[str, object], dict[str, object] | None]:
    """Build the real-start status payload and optional updated latch."""

    if not required:
        return real_start_not_required_payload(
            hold_active=hold_active,
            latched_payload=latched_payload,
        )

    attempt = build_required_real_start_attempt(
        RequiredRealStartInputs(
            env_float=env_float,
            hold_active=hold_active,
            base_depth_m=base_depth_m,
            bar30_depth_m=bar30_depth_m,
            base_xy_m=base_xy_m,
            current_rpy_rad=current_rpy_rad,
            release_linear_velocity_body=release_linear_velocity_body,
            release_angular_velocity_body=release_angular_velocity_body,
            model_density=model_density,
            depth_tolerance_m=depth_tolerance_m,
            attitude_tolerance_rad=attitude_tolerance_rad,
            velocity_tolerance_mps=velocity_tolerance_mps,
        )
    )
    return resolve_required_real_start_payload(
        attempt=attempt,
        latched_payload=latched_payload,
        hold_active=hold_active,
    )


__all__ = ["build_real_start_status_payload"]

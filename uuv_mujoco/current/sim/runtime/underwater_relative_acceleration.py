"""Relative body-acceleration tracking for underwater hydrodynamics."""

from __future__ import annotations

from typing import Any
import math

import numpy as np


def reset_relative_acceleration_history(runtime: Any) -> None:
    """Clear finite-difference history after a discontinuous state change."""

    runtime.prev_rel_nu_body[:] = 0.0
    runtime.prev_rel_nu_valid = False
    runtime.prev_rel_sample_time_s = float("nan")


def reset_relative_acceleration_on_hold_transition(
    runtime: Any,
    *,
    hold_active: bool,
) -> bool:
    """Reset history once when an initial-depth hold is released."""

    active = bool(hold_active)
    previous = bool(
        getattr(runtime, "previous_initial_depth_hold_active", active)
    )
    runtime.previous_initial_depth_hold_active = active
    if previous and not active:
        reset_relative_acceleration_history(runtime)
        return True
    return False


def update_relative_acceleration(runtime: Any, nu_rel_body: np.ndarray, dt: float) -> np.ndarray:
    hyd = runtime.hydrodynamics
    need_rel_acc_body = runtime.use_custom_hydrodynamics or hyd.fossen_residual_added_mass_active
    if need_rel_acc_body:
        now_s = float(getattr(getattr(runtime, "data", None), "time", float("nan")))
        previous_time_s = float(getattr(runtime, "prev_rel_sample_time_s", float("nan")))
        valid = bool(getattr(runtime, "prev_rel_nu_valid", False))
        continuous_time = (
            dt > 0.0
            and valid
            and math.isfinite(now_s)
            and math.isfinite(previous_time_s)
            and now_s > previous_time_s
            and (now_s - previous_time_s) <= max(4.0 * float(dt), 0.05)
        )
        if continuous_time:
            rel_acc_body = (nu_rel_body - runtime.prev_rel_nu_body) / (now_s - previous_time_s)
        else:
            # Seed on first use, pause release, time reset, or a long gap.
            # Treating a non-zero initial velocity/current as acceleration
            # produced a one-step added-mass impulse (hundreds of newtons at
            # dt=5 ms) even though no physical acceleration had occurred.
            rel_acc_body = np.zeros(6, dtype=np.float64)
        runtime.prev_rel_nu_body[:] = nu_rel_body
        runtime.prev_rel_nu_valid = True
        runtime.prev_rel_sample_time_s = now_s
        return rel_acc_body
    reset_relative_acceleration_history(runtime)
    return np.zeros(6, dtype=np.float64)


__all__ = [
    "reset_relative_acceleration_history",
    "reset_relative_acceleration_on_hold_transition",
    "update_relative_acceleration",
]

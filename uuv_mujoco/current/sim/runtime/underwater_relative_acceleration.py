"""Relative body-acceleration tracking for underwater hydrodynamics."""

from __future__ import annotations

from typing import Any
import math

import numpy as np


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
            rel_acc_body = (nu_rel_body - runtime.prev_rel_nu_body) / max(dt, 1e-6)
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
    runtime.prev_rel_nu_body[:] = 0.0
    runtime.prev_rel_nu_valid = False
    runtime.prev_rel_sample_time_s = float("nan")
    return np.zeros(6, dtype=np.float64)


__all__ = ["update_relative_acceleration"]

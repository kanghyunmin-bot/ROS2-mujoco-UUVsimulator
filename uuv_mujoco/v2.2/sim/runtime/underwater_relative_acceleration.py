"""Relative body-acceleration tracking for underwater hydrodynamics."""

from __future__ import annotations

from typing import Any

import numpy as np


def update_relative_acceleration(runtime: Any, nu_rel_body: np.ndarray, dt: float) -> np.ndarray:
    hyd = runtime.hydrodynamics
    need_rel_acc_body = runtime.use_custom_hydrodynamics or hyd.fossen_residual_added_mass_active
    if need_rel_acc_body:
        if dt > 0.0:
            rel_acc_body = (nu_rel_body - runtime.prev_rel_nu_body) / max(dt, 1e-6)
        else:
            rel_acc_body = np.zeros(6, dtype=np.float64)
        runtime.prev_rel_nu_body = nu_rel_body.copy()
        return rel_acc_body
    runtime.prev_rel_nu_body[:] = 0.0
    return np.zeros(6, dtype=np.float64)


__all__ = ["update_relative_acceleration"]

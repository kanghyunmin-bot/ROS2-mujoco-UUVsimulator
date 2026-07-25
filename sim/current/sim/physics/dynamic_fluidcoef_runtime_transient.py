"""Transient onset/decay helpers for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

from typing import Any

import numpy as np


def apply_dynamic_fluidcoef_transient(
    runtime: Any,
    idx: np.ndarray,
    blend: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    transient = np.ones((idx.size, 5), dtype=np.float64)
    reset_mask = np.zeros((idx.size, 5), dtype=bool)
    if not runtime.transient_enabled or not np.any(runtime.log_decay_mask):
        return transient, reset_mask

    prev_blend = runtime.prev_blend[idx, :]
    active = runtime.transient_active[idx, :]
    age_s = runtime.transient_age_s[idx, :]
    drag_decay_mask = runtime.log_decay_mask.reshape(1, 5)

    rearmed = (blend <= runtime.transient_rearm_load) & drag_decay_mask
    active[rearmed] = False
    age_s[rearmed] = np.inf

    starts = (blend >= runtime.transient_reset_load) & (~active) & drag_decay_mask
    retriggers = (
        active
        & (blend >= runtime.transient_reset_load)
        & ((blend - prev_blend) >= runtime.transient_retrigger_delta)
        & drag_decay_mask
    )
    reset_mask = starts | retriggers
    active[reset_mask] = True
    age_s[reset_mask] = 0.0

    still_active = active & (~reset_mask) & drag_decay_mask
    age_s[still_active] = np.where(
        np.isfinite(age_s[still_active]),
        age_s[still_active] + runtime.update_dt,
        0.0,
    )

    decay = dynamic_fluidcoef_decay(runtime, age_s, active)
    transient[:, runtime.log_decay_mask] = decay[:, runtime.log_decay_mask]
    runtime.transient_active[idx, :] = active
    runtime.transient_age_s[idx, :] = age_s
    runtime.prev_blend[idx, :] = blend
    return transient, reset_mask


def dynamic_fluidcoef_decay(runtime: Any, age_s: np.ndarray, active: np.ndarray) -> np.ndarray:
    if runtime.transient_mode in {"linear"}:
        decay = np.maximum(
            runtime.transient_floor,
            1.0 - age_s / runtime.transient_decay_s,
        )
    else:
        decay = 1.0 / (
            1.0 + np.log1p(np.maximum(age_s, 0.0) / runtime.transient_decay_s)
        )
        decay = np.maximum(runtime.transient_floor, decay)
    decay[~active] = 0.0
    return np.clip(decay, 0.0, 1.0)


__all__ = ["apply_dynamic_fluidcoef_transient", "dynamic_fluidcoef_decay"]

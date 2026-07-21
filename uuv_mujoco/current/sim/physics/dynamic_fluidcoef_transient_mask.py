"""Transient coefficient-mask knobs for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np


DEFAULT_LOG_DECAY_COEFFICIENTS = np.array([1, 1, 1, 0, 0], dtype=np.float64)


def configure_dynamic_fluidcoef_transient_mask(
    runtime,
    *,
    env_float: Callable[[str, float], float],
    to_float_array: Callable[[object], np.ndarray | None],
) -> None:
    log_decay_coefficients = to_float_array(
        runtime.cfg.get("log_decay_coefficients", DEFAULT_LOG_DECAY_COEFFICIENTS)
    )
    if log_decay_coefficients is None or log_decay_coefficients.size != 5:
        log_decay_coefficients = DEFAULT_LOG_DECAY_COEFFICIENTS.copy()
    runtime.log_decay_mask = np.asarray(log_decay_coefficients, dtype=np.float64) > 0.5
    runtime.lift_alpha = float(
        np.clip(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_LIFT_ALPHA",
                float(runtime.cfg.get("lift_smoothing_alpha", runtime.alpha)),
            ),
            0.01,
            1.0,
        )
    )


__all__ = ["DEFAULT_LOG_DECAY_COEFFICIENTS", "configure_dynamic_fluidcoef_transient_mask"]

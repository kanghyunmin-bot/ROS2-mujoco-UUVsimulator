"""Update-rate and smoothing knobs for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np


def configure_dynamic_fluidcoef_update_knobs(
    runtime,
    *,
    env_float: Callable[[str, float], float],
) -> None:
    runtime.update_hz = float(
        np.clip(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_HZ",
                float(runtime.cfg.get("update_hz", 20.0)),
            ),
            0.5,
            500.0,
        )
    )
    runtime.update_dt = 1.0 / max(runtime.update_hz, 1.0e-6)
    runtime.alpha = float(
        np.clip(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_ALPHA",
                float(runtime.cfg.get("smoothing_alpha", 0.25)),
            ),
            0.01,
            1.0,
        )
    )
    runtime.ref_speed = float(
        max(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_REFERENCE_SPEED_MPS",
                float(runtime.cfg.get("reference_speed_mps", 0.3)),
            ),
            1.0e-3,
        )
    )
    runtime.ref_angular = float(
        max(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_REFERENCE_ANGULAR_RPS",
                float(runtime.cfg.get("reference_angular_rps", 0.6)),
            ),
            1.0e-3,
        )
    )


__all__ = ["configure_dynamic_fluidcoef_update_knobs"]

"""Transient threshold knobs for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np


def configure_dynamic_fluidcoef_transient_thresholds(
    runtime,
    *,
    env_float: Callable[[str, float], float],
) -> None:
    runtime.transient_decay_s = float(
        max(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_TRANSIENT_DECAY_S",
                float(runtime.cfg.get("transient_decay_s", 2.5)),
            ),
            1.0e-3,
        )
    )
    runtime.transient_attack_alpha = float(
        np.clip(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_TRANSIENT_ATTACK_ALPHA",
                float(runtime.cfg.get("transient_attack_alpha", 1.0)),
            ),
            0.01,
            1.0,
        )
    )
    runtime.transient_reset_load = float(
        np.clip(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_TRANSIENT_RESET_LOAD",
                float(runtime.cfg.get("transient_reset_load", 0.06)),
            ),
            0.0,
            1.0,
        )
    )
    runtime.transient_rearm_load = float(
        np.clip(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_TRANSIENT_REARM_LOAD",
                float(runtime.cfg.get("transient_rearm_load", 0.02)),
            ),
            0.0,
            1.0,
        )
    )
    runtime.transient_rearm_load = min(runtime.transient_rearm_load, runtime.transient_reset_load)
    runtime.transient_retrigger_delta = float(
        np.clip(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_TRANSIENT_RETRIGGER_DELTA",
                float(runtime.cfg.get("transient_retrigger_delta", 0.15)),
            ),
            0.0,
            1.0,
        )
    )
    runtime.transient_floor = float(
        np.clip(
            env_float(
                "UUV_DYNAMIC_FLUIDCOEF_TRANSIENT_FLOOR",
                float(runtime.cfg.get("transient_floor", 0.0)),
            ),
            0.0,
            1.0,
        )
    )


__all__ = ["configure_dynamic_fluidcoef_transient_thresholds"]

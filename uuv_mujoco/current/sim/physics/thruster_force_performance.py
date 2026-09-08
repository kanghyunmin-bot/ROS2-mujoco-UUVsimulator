"""T200/direct performance-curve force conversion."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np


def pwm_to_force_from_performance(norm_cmd: float, perf_cfg: Mapping[str, Any]) -> float:
    """Map normalized command [-1, 1] to force using a PWM performance curve."""
    pwm = float(np.clip(norm_cmd, -1.0, 1.0) * 400.0 + 1500.0)
    # Basic ESC signal deadband. Do not renormalize the remaining PWM range:
    # the measured curve already contains ESC/motor startup behavior.
    if abs(pwm - 1500.0) <= 25.0:
        return 0.0
    return float(np.interp(pwm, perf_cfg["pwm"], perf_cfg["force"]))


def performance_curve_active(perf_cfg: Mapping[str, Any]) -> bool:
    force_curve = perf_cfg.get("force")
    return bool(perf_cfg.get("active") and getattr(force_curve, "size", 0) > 0)


def force_from_performance_curve(
    *,
    name: str,
    command_shaped: float,
    gain: float,
    perf_cfg: Mapping[str, Any],
    thruster_direct_scale: Mapping[str, float],
) -> float:
    if perf_cfg.get("direct"):
        return float(
            pwm_to_force_from_performance(command_shaped, perf_cfg)
            * float(thruster_direct_scale.get(name, 1.0))
        )
    return float(pwm_to_force_from_performance(command_shaped, perf_cfg) * gain)


__all__ = [
    "force_from_performance_curve",
    "performance_curve_active",
    "pwm_to_force_from_performance",
]

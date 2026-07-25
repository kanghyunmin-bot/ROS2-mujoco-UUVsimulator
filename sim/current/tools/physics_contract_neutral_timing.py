"""Timing helpers for neutral-PWM physics contract simulations."""

from __future__ import annotations

import math


def neutral_step_count(duration_s: float, dt: float) -> int:
    return int(max(math.ceil(float(duration_s) / max(float(dt), 1.0e-9)), 1))


def neutral_sample_every(dt: float, sample_dt_s: float = 0.05) -> int:
    return max(int(round(float(sample_dt_s) / max(float(dt), 1.0e-9))), 1)


__all__ = ["neutral_sample_every", "neutral_step_count"]

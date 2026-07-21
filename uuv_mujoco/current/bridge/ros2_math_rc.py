"""RC channel normalization helpers used by command contracts."""

from __future__ import annotations

import numpy as np


def clamp_rc_channel(value: int) -> int:
    return max(0, min(17, int(value)))


def rc_channel_value(channels, index: int) -> int:
    return int(channels[index]) if index < len(channels) else 0


def rc_to_norm(pwm_value: int, *, pwm_span: float, invert: bool = False) -> float:
    pwm = int(pwm_value)
    if pwm <= 0 or pwm < 800 or pwm > 2200:
        return 0.0
    norm = (float(pwm) - 1500.0) / max(float(pwm_span), 1e-6)
    if invert:
        norm = -norm
    return float(np.clip(norm, -1.0, 1.0))

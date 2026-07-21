"""ArduSub joystick and MANUAL heave RC conversion helpers."""

from __future__ import annotations

from .rc_constants import PWM_CENTER
from .rc_math import clamp_float


def effective_joystick_gain(
    *,
    gain_default: float,
    gain_min: float,
    gain_max: float,
    gain_steps: int,
) -> float:
    """Mirror ArduSub joystick.cpp init_joystick() gain selection."""
    steps = max(1, int(gain_steps))
    default = float(gain_default)
    min_gain = float(gain_min)
    max_gain = float(gain_max)
    if steps == 1 or (default < max_gain + 0.01 and default > min_gain - 0.01):
        gain = clamp_float(default, min_gain, max_gain)
    else:
        gain = min_gain + (steps / 2.0 - 1.0) * (max_gain - min_gain) / float(steps - 1)
    return clamp_float(gain, 0.1, 1.0)


def manual_heave_to_rc3_pwm(
    value: float,
    *,
    pwm_neutral: int = PWM_CENTER,
    rc_min: int,
    rc_max: int,
    gain: float,
    throttle_gain: float,
) -> int:
    """Mirror ArduSub MANUAL_CONTROL.z -> RC3 override conversion."""
    gain_f = clamp_float(gain, 0.1, 1.0)
    throttle_scale = 0.8 * gain_f * float(throttle_gain)
    throttle_base = float(pwm_neutral) - 500.0 * throttle_scale
    manual_z = 500.0 + clamp_float(value, -1.0, 1.0) * 500.0
    return int(round(clamp_float(manual_z * throttle_scale + throttle_base, int(rc_min), int(rc_max))))


__all__ = ["effective_joystick_gain", "manual_heave_to_rc3_pwm"]

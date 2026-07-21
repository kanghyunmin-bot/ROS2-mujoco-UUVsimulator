"""GUI RC PWM conversion helpers."""

from __future__ import annotations

from .config import RC_NEUTRAL_PWM, RC_PWM_SPAN, RC_VALID_MAX_PWM, RC_VALID_MIN_PWM
from .gui_axis_normalization import clamp_axis


def axis_to_pwm(value: float) -> int:
    """Map a normalized joystick axis in [-1, 1] to ArduSub RC PWM."""
    return int(round(RC_NEUTRAL_PWM + clamp_axis(value) * RC_PWM_SPAN))


def heave_axis_to_rc3_pwm(value: float) -> int:
    """Map GUI heave to ArduSub-4.1.2 RC3."""
    return axis_to_pwm(value)


def valid_rc_pwm(value: int) -> bool:
    return RC_VALID_MIN_PWM <= int(value) <= RC_VALID_MAX_PWM


__all__ = ["axis_to_pwm", "heave_axis_to_rc3_pwm", "valid_rc_pwm"]

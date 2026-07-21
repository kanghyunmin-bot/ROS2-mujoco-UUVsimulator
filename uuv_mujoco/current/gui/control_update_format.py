"""Lightweight format helpers for the GUI telemetry refresh loop."""

from __future__ import annotations

import math

from sim.contracts import althold_climb_rate_from_rc3_pwm, effective_joystick_gain, manual_heave_to_rc3_pwm

from .config import (
    AXIS_MAX,
    AXIS_MIN,
    GUI_PILOT_CONTROL_MODE,
    PILOT_CONTROL_RC_OVERRIDE,
    RC_NEUTRAL_PWM,
    RC_PWM_SPAN,
    REAL_JS_GAIN_DEFAULT,
    REAL_JS_GAIN_MAX,
    REAL_JS_GAIN_MIN,
    REAL_JS_GAIN_STEPS,
    REAL_JS_THR_GAIN,
    REAL_PILOT_SPEED_DN,
    REAL_PILOT_SPEED_UP,
    REAL_RC3_DZ,
    REAL_RC3_MAX,
    REAL_RC3_MIN,
    REAL_RC3_TRIM,
)


def format_age(age_s: float) -> str:
    if not math.isfinite(age_s):
        return "n/a"
    if age_s < 1.0:
        return f"{age_s * 1000.0:.0f} ms"
    return f"{age_s:.1f} s"


def axis_to_pwm(value: float) -> int:
    return int(round(RC_NEUTRAL_PWM + clamp_axis(value) * RC_PWM_SPAN))


def pilot_heave_axis_summary(value: float, *, mode: str = GUI_PILOT_CONTROL_MODE) -> tuple[int, float]:
    rc3_pwm = axis_to_pwm(value) if mode == PILOT_CONTROL_RC_OVERRIDE else manual_heave_axis_to_pwm(value)
    return rc3_pwm, althold_climb_rate_from_rc3_pwm(
        rc3_pwm,
        rc_min=REAL_RC3_MIN,
        rc_max=REAL_RC3_MAX,
        rc_trim=REAL_RC3_TRIM,
        rc_deadzone=REAL_RC3_DZ,
        pilot_speed_up=REAL_PILOT_SPEED_UP,
        pilot_speed_dn=REAL_PILOT_SPEED_DN,
        gain=joystick_gain(),
    )


def manual_heave_axis_to_pwm(value: float) -> int:
    return manual_heave_to_rc3_pwm(
        value,
        pwm_neutral=RC_NEUTRAL_PWM,
        rc_min=REAL_RC3_MIN,
        rc_max=REAL_RC3_MAX,
        gain=joystick_gain(),
        throttle_gain=REAL_JS_THR_GAIN,
    )


def joystick_gain() -> float:
    return effective_joystick_gain(
        gain_default=REAL_JS_GAIN_DEFAULT,
        gain_min=REAL_JS_GAIN_MIN,
        gain_max=REAL_JS_GAIN_MAX,
        gain_steps=REAL_JS_GAIN_STEPS,
    )


def clamp_axis(value: float) -> float:
    return max(AXIS_MIN, min(AXIS_MAX, float(value)))


__all__ = ["axis_to_pwm", "format_age", "pilot_heave_axis_summary"]

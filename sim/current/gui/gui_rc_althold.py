"""GUI ArduSub joystick and ALT_HOLD diagnostic helpers."""

from __future__ import annotations

from sim.contracts import (
    althold_climb_rate_from_rc3_pwm as contract_althold_climb_rate_from_rc3_pwm,
    effective_joystick_gain as contract_effective_joystick_gain,
    manual_heave_to_rc3_pwm as contract_manual_heave_to_rc3_pwm,
)

from .config import (
    GUI_PILOT_CONTROL_MODE,
    PILOT_CONTROL_RC_OVERRIDE,
    RC_NEUTRAL_PWM,
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
from .gui_axis_normalization import clamp
from .gui_rc_pwm import heave_axis_to_rc3_pwm


def effective_js_gain(
    *,
    gain_default: float = REAL_JS_GAIN_DEFAULT,
    gain_min: float = REAL_JS_GAIN_MIN,
    gain_max: float = REAL_JS_GAIN_MAX,
    gain_steps: int = REAL_JS_GAIN_STEPS,
) -> float:
    """Mirror ArduSub joystick.cpp init_joystick() gain selection."""
    return contract_effective_joystick_gain(
        gain_default=gain_default,
        gain_min=gain_min,
        gain_max=gain_max,
        gain_steps=gain_steps,
    )


def manual_heave_axis_to_rc3_pwm(
    value: float,
    *,
    gain: float | None = None,
    throttle_gain: float = REAL_JS_THR_GAIN,
) -> int:
    """Mirror ArduSub MANUAL_CONTROL.z -> RC3 conversion for GUI diagnostics."""
    gain = effective_js_gain() if gain is None else clamp(float(gain), 0.1, 1.0)
    return contract_manual_heave_to_rc3_pwm(
        value,
        pwm_neutral=RC_NEUTRAL_PWM,
        rc_min=REAL_RC3_MIN,
        rc_max=REAL_RC3_MAX,
        gain=gain,
        throttle_gain=throttle_gain,
    )


def althold_level_climb_rate_from_rc3_pwm(
    rc3_pwm: float,
    *,
    gain: float | None = None,
    rc_min: int = REAL_RC3_MIN,
    rc_max: int = REAL_RC3_MAX,
    rc_trim: int = REAL_RC3_TRIM,
    rc_deadzone: int = REAL_RC3_DZ,
    pilot_speed_up: float = REAL_PILOT_SPEED_UP,
    pilot_speed_dn: float = REAL_PILOT_SPEED_DN,
) -> float:
    """Approximate ArduSub ALT_HOLD target climb rate for a level vehicle."""
    gain = effective_js_gain() if gain is None else clamp(float(gain), 0.1, 1.0)
    return contract_althold_climb_rate_from_rc3_pwm(
        rc3_pwm,
        rc_min=rc_min,
        rc_max=rc_max,
        rc_trim=rc_trim,
        rc_deadzone=rc_deadzone,
        pilot_speed_up=pilot_speed_up,
        pilot_speed_dn=pilot_speed_dn,
        gain=gain,
    )


def pilot_heave_axis_summary(value: float, *, mode: str = GUI_PILOT_CONTROL_MODE) -> tuple[int, float]:
    """Return expected RC3 PWM and level ALT_HOLD climb target for GUI stick heave."""
    if mode == PILOT_CONTROL_RC_OVERRIDE:
        rc3_pwm = heave_axis_to_rc3_pwm(value)
    else:
        rc3_pwm = manual_heave_axis_to_rc3_pwm(value)
    return rc3_pwm, althold_level_climb_rate_from_rc3_pwm(rc3_pwm)


__all__ = [
    "effective_js_gain",
    "manual_heave_axis_to_rc3_pwm",
    "althold_level_climb_rate_from_rc3_pwm",
    "pilot_heave_axis_summary",
]

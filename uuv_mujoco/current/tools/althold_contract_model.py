"""ArduSub ALT_HOLD heave contract constants and equations."""

from __future__ import annotations

import math


RC_MIN = 1100
RC_MAX = 1900
RC_NEUTRAL = 1500
RC3_TRIM = 1100
RC3_DZ = 30
PILOT_SPEED_UP_CM_S = 100.0
PILOT_SPEED_DN_CM_S = 0.0
JS_GAIN_DEFAULT = 0.5
JS_GAIN_MIN = 0.25
JS_GAIN_MAX = 1.0
JS_GAIN_STEPS = 1
MODE_NAMES = {
    2: "ALT_HOLD",
    19: "MANUAL",
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def effective_js_gain() -> float:
    steps = max(1, int(JS_GAIN_STEPS))
    if steps == 1 or (JS_GAIN_DEFAULT < JS_GAIN_MAX + 0.01 and JS_GAIN_DEFAULT > JS_GAIN_MIN - 0.01):
        gain = clamp(JS_GAIN_DEFAULT, JS_GAIN_MIN, JS_GAIN_MAX)
    else:
        gain = JS_GAIN_MIN + (steps / 2.0 - 1.0) * (JS_GAIN_MAX - JS_GAIN_MIN) / float(steps - 1)
    return clamp(gain, 0.1, 1.0)


def pilot_speed_dn() -> float:
    return abs(PILOT_SPEED_DN_CM_S) if PILOT_SPEED_DN_CM_S != 0.0 else abs(PILOT_SPEED_UP_CM_S)


def rc3_to_expected_althold_climb(rc3_pwm: float) -> float:
    if not math.isfinite(rc3_pwm):
        return math.nan
    if rc3_pwm < RC3_TRIM:
        norm = 0.0 if RC_MIN >= RC3_TRIM else (rc3_pwm - RC3_TRIM) / float(RC3_TRIM - RC_MIN)
    else:
        norm = 0.0 if RC_MAX <= RC3_TRIM else (rc3_pwm - RC3_TRIM) / float(RC_MAX - RC3_TRIM)
    norm = clamp(norm, -1.0, 1.0)
    earth_z = 2.0 * (-0.5 + norm)
    throttle_control = 500.0 + PILOT_SPEED_UP_CM_S * earth_z
    center = (RC_MAX + RC_MIN) / 2.0
    target = throttle_control - center + 1000.0
    if abs(target) < RC3_DZ * effective_js_gain():
        target = 0.0
    return clamp(target, -pilot_speed_dn(), PILOT_SPEED_UP_CM_S)


__all__ = [
    "JS_GAIN_DEFAULT",
    "JS_GAIN_MAX",
    "JS_GAIN_MIN",
    "JS_GAIN_STEPS",
    "MODE_NAMES",
    "PILOT_SPEED_DN_CM_S",
    "PILOT_SPEED_UP_CM_S",
    "RC3_DZ",
    "RC3_TRIM",
    "RC_MAX",
    "RC_MIN",
    "RC_NEUTRAL",
    "clamp",
    "effective_js_gain",
    "pilot_speed_dn",
    "rc3_to_expected_althold_climb",
]

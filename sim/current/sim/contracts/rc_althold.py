"""ALT_HOLD RC3 climb-rate approximation contract."""

from __future__ import annotations

from .rc_math import clamp_float


def althold_climb_rate_from_rc3_pwm(
    rc3_pwm: float,
    *,
    rc_min: int,
    rc_max: int,
    rc_trim: int,
    rc_deadzone: int,
    pilot_speed_up: float,
    pilot_speed_dn: float,
    gain: float,
) -> float:
    """Approximate ArduSub ALT_HOLD target climb rate for a level vehicle."""
    pwm = float(rc3_pwm)
    min_pwm = int(rc_min)
    max_pwm = int(rc_max)
    trim_pwm = int(rc_trim)
    if pwm < trim_pwm:
        norm = 0.0 if min_pwm >= trim_pwm else (pwm - trim_pwm) / float(trim_pwm - min_pwm)
    else:
        norm = 0.0 if max_pwm <= trim_pwm else (pwm - trim_pwm) / float(max_pwm - trim_pwm)
    norm = clamp_float(norm, -1.0, 1.0)
    earth_z = 2.0 * (-0.5 + norm)
    throttle_control = 500.0 + float(pilot_speed_up) * earth_z
    center = (float(max_pwm) + float(min_pwm)) / 2.0
    target_climb = throttle_control - center + 1000.0
    if abs(target_climb) < float(rc_deadzone) * clamp_float(gain, 0.1, 1.0):
        target_climb = 0.0
    speed_down = abs(float(pilot_speed_dn)) if float(pilot_speed_dn) != 0.0 else abs(float(pilot_speed_up))
    return clamp_float(target_climb, -speed_down, float(pilot_speed_up))


__all__ = ["althold_climb_rate_from_rc3_pwm"]

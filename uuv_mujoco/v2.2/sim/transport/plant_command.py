"""Pure PWM classification helpers for MuJoCo plant command handling."""

from __future__ import annotations

PWM_NEUTRAL = 1500
PWM_IGNORE = 65535
PWM_ACTIVE_TOLERANCE = 12
PWM_ALL_MIN_THRESHOLD = 1120


def active_pwm_values(pwm_values: list[int], *, channels: int = 8) -> list[int]:
    out: list[int] = []
    for value in pwm_values[: int(channels)]:
        ivalue = int(value)
        if ivalue > 0 and ivalue != PWM_IGNORE:
            out.append(ivalue)
    return out


def has_nonneutral_pwm(
    pwm_values: list[int],
    *,
    center: int = PWM_NEUTRAL,
    tolerance: int = PWM_ACTIVE_TOLERANCE,
    channels: int = 8,
) -> bool:
    return any(abs(value - int(center)) > int(tolerance) for value in active_pwm_values(pwm_values, channels=channels))


def all_active_outputs_at_min(
    pwm_values: list[int],
    *,
    threshold: int = PWM_ALL_MIN_THRESHOLD,
    channels: int = 8,
) -> bool:
    active = active_pwm_values(pwm_values, channels=channels)
    return len(active) >= int(channels) and all(value <= int(threshold) for value in active[: int(channels)])


def neutral_pwm_frame(length: int, *, center: int = PWM_NEUTRAL, min_channels: int = 8) -> list[int]:
    return [int(center)] * max(int(min_channels), int(length))


__all__ = [
    "PWM_NEUTRAL",
    "PWM_IGNORE",
    "PWM_ACTIVE_TOLERANCE",
    "PWM_ALL_MIN_THRESHOLD",
    "active_pwm_values",
    "has_nonneutral_pwm",
    "all_active_outputs_at_min",
    "neutral_pwm_frame",
]

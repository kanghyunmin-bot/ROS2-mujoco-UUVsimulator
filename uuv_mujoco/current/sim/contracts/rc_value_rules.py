"""Marker and PWM value rules for RC override contracts."""

from __future__ import annotations

from .rc_constants import (
    RC_EXTENSION_NO_CHANGE_VALUE,
    RC_IGNORE_VALUE,
    RC_RELEASE_VALUE,
    RC_VALID_MAX_PWM,
    RC_VALID_MIN_PWM,
)


def valid_pwm(value: int) -> bool:
    return RC_VALID_MIN_PWM <= int(value) <= RC_VALID_MAX_PWM


def sanitize_primary_motion_value(value: int, *, center_pwm: int) -> int:
    return int(value) if valid_pwm(value) else int(center_pwm)


def sanitize_legacy_override_value(value: int, *, center_pwm: int) -> int:
    if value in (RC_RELEASE_VALUE, RC_IGNORE_VALUE) or valid_pwm(value):
        return int(value)
    return int(center_pwm)


def sanitize_extension_value(value: int) -> int:
    if value in (RC_RELEASE_VALUE, RC_EXTENSION_NO_CHANGE_VALUE, RC_IGNORE_VALUE) or valid_pwm(value):
        return int(value)
    return RC_RELEASE_VALUE


def normalize_legacy_override_value(value: int) -> int:
    if value <= 0 or value == RC_EXTENSION_NO_CHANGE_VALUE:
        return RC_RELEASE_VALUE
    if value == RC_IGNORE_VALUE:
        return RC_IGNORE_VALUE
    return int(value) if valid_pwm(value) else RC_RELEASE_VALUE


def normalize_extension_override_value(value: int) -> int:
    if value in (RC_RELEASE_VALUE, RC_EXTENSION_NO_CHANGE_VALUE, RC_IGNORE_VALUE) or valid_pwm(value):
        return int(value)
    return RC_RELEASE_VALUE


__all__ = [
    "normalize_extension_override_value",
    "normalize_legacy_override_value",
    "sanitize_extension_value",
    "sanitize_legacy_override_value",
    "sanitize_primary_motion_value",
    "valid_pwm",
]

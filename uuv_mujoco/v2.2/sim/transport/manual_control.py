"""Pure helpers for MAVLink MANUAL_CONTROL pilot input."""

from __future__ import annotations


def _clip_int(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, int(value)))


def manual_axis_to_int(value: float) -> int:
    """Convert normalized or raw MANUAL_CONTROL lateral/yaw axis to [-1000, 1000]."""
    value_f = float(value)
    if -1.0 <= value_f <= 1.0:
        value_f *= 1000.0
    return _clip_int(round(value_f), -1000, 1000)


def manual_thrust_to_int(value: float) -> int:
    """Convert normalized or raw MANUAL_CONTROL thrust axis to [0, 1000]."""
    value_f = float(value)
    if -1.0 <= value_f <= 1.0:
        value_f = 500.0 + value_f * 500.0
    return _clip_int(round(value_f), 0, 1000)


def manual_axes_are_near_neutral(
    *,
    x: int,
    y: int,
    z: int,
    r: int,
    tolerance: int = 50,
) -> bool:
    """Return true when ArduSub joystick priming should accept the current axes as neutral."""
    tol = abs(int(tolerance))
    return abs(int(x)) <= tol and abs(int(y)) <= tol and abs(int(z) - 500) <= tol and abs(int(r)) <= tol

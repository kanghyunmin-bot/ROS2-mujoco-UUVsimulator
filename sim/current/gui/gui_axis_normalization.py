"""Small GUI axis normalization helpers without ROS runtime imports."""

from __future__ import annotations

from .config import AXIS_MAX, AXIS_MIN


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp_axis(value: float) -> float:
    return clamp(float(value), AXIS_MIN, AXIS_MAX)


def normalize_axes(
    *,
    forward: float,
    lateral: float,
    heave: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    return (
        clamp_axis(forward),
        clamp_axis(lateral),
        clamp_axis(heave),
        clamp_axis(yaw),
    )


__all__ = ["clamp", "clamp_axis", "normalize_axes"]

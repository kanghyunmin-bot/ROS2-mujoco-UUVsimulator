"""Value helpers for runtime direct-command state."""

from __future__ import annotations


COMMAND_FIELDS = ("forward", "sway", "yaw", "heave")


def clamp_command_value(value: float, max_value: float) -> float:
    limit = abs(float(max_value))
    return max(-limit, min(limit, float(value)))


def clamped_command_values(
    *,
    forward: float,
    sway: float,
    yaw: float,
    heave: float,
    max_value: float,
) -> dict[str, float]:
    return {
        "forward": clamp_command_value(forward, max_value),
        "sway": clamp_command_value(sway, max_value),
        "yaw": clamp_command_value(yaw, max_value),
        "heave": clamp_command_value(heave, max_value),
    }


def normalized_command_tuple(values: dict[str, float], *, max_value: float) -> tuple[float, float, float, float]:
    scale = max(float(max_value), 1.0e-6)
    return tuple(clamp_command_value(float(values[name]) / scale, 1.0) for name in COMMAND_FIELDS)


__all__ = ["COMMAND_FIELDS", "clamp_command_value", "clamped_command_values", "normalized_command_tuple"]

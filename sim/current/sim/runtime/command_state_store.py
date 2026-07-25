"""Storage operations for runtime direct-command state."""

from __future__ import annotations

from sim.runtime.command_state_values import clamped_command_values, normalized_command_tuple


def default_command_values() -> dict[str, float]:
    return {"forward": 0.0, "heave": 0.0, "yaw": 0.0, "sway": 0.0}


def clamped_command_update(
    *,
    forward: float,
    sway: float,
    yaw: float,
    heave: float,
    max_value: float,
) -> dict[str, float]:
    return clamped_command_values(
        forward=forward,
        sway=sway,
        yaw=yaw,
        heave=heave,
        max_value=max_value,
    )


def normalized_command_snapshot(
    values: dict[str, float],
    *,
    max_value: float,
) -> tuple[float, float, float, float]:
    return normalized_command_tuple(values, max_value=max_value)


def command_recently_active(*, last_wall: float, now_wall: float, timeout_s: float) -> bool:
    return float(last_wall) > 0.0 and (float(now_wall) - float(last_wall)) <= float(timeout_s)


__all__ = [
    "clamped_command_update",
    "command_recently_active",
    "default_command_values",
    "normalized_command_snapshot",
]

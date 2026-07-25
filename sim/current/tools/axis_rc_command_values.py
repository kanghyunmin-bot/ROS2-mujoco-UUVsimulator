"""Normalized command helpers for axis RC checks."""

from __future__ import annotations


def clamp_axis_command(command: float) -> float:
    return max(-1.0, min(1.0, float(command)))


def axis_command_value(axis: str | None, command: float, *, invert_heave_rc: bool = False) -> float:
    command_value = clamp_axis_command(command)
    if axis == "heave" and invert_heave_rc:
        return -command_value
    return command_value


__all__ = ["axis_command_value", "clamp_axis_command"]

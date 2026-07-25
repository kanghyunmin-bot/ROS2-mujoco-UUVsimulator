"""Boolean parser for ROS2/MAVROS command override payloads."""

from __future__ import annotations


def _parse_command_bool(value: object, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if not text:
        return bool(default)
    return text in {"1", "true", "yes", "on", "arm", "armed"}


__all__ = ["_parse_command_bool"]

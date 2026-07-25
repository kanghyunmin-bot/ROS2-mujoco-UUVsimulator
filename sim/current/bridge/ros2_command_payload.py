"""Command payload parsing helpers for ROS2/MAVROS bridge callbacks."""

from __future__ import annotations

import json

from .ros2_command_bool import _parse_command_bool
from .ros2_command_payload_tokens import _parse_payload_tokens


def _parse_command_override_payload(text: str) -> dict[str, object]:
    raw = str(text or "").strip()
    if not raw:
        return {}
    try:
        payload = json.loads(raw)
        return payload if isinstance(payload, dict) else {}
    except Exception:
        pass

    return _parse_payload_tokens(raw)


def bind_command_payload_parsers(cls: type[object]) -> None:
    """Bind command-payload parsers without importing the full runtime bridge."""

    cls._parse_command_bool = staticmethod(_parse_command_bool)
    cls._parse_command_override_payload = staticmethod(_parse_command_override_payload)


__all__ = [
    "_parse_command_bool",
    "_parse_command_override_payload",
    "bind_command_payload_parsers",
]

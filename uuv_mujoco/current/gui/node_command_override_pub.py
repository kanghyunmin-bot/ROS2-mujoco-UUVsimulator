"""Command override publisher helper."""

from __future__ import annotations

import json

from .runtime import String, os


def _publish_command_override(self, payload: dict) -> bool:
    try:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        burst_count = _command_override_burst_count()
        for _ in range(burst_count):
            self._command_override_pub.publish(msg)
        return True
    except Exception as exc:
        self._push_event(f"command_override publish failed: {exc}")
        return False


def _command_override_burst_count() -> int:
    try:
        raw = int(float(os.environ.get("UUV_GUI_COMMAND_OVERRIDE_BURST_COUNT", "3")))
    except (TypeError, ValueError):
        raw = 3
    return max(1, min(8, raw))


__all__ = ["_command_override_burst_count", "_publish_command_override"]

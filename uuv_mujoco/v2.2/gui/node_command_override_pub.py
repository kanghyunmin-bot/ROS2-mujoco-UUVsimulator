"""Command override publisher helper."""

from __future__ import annotations

import json

from .runtime import String


def _publish_command_override(self, payload: dict) -> bool:
    try:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._command_override_pub.publish(msg)
        return True
    except Exception as exc:
        self._push_event(f"command_override publish failed: {exc}")
        return False


__all__ = ["_publish_command_override"]

"""Message publish helper for real-start status payloads."""

from __future__ import annotations

import json


def publish_real_start_status(publisher, string_type, payload: dict[str, object]) -> bool:
    """Publish one real-start status payload."""
    if publisher is None or string_type is None:
        return False
    try:
        msg = string_type()
        msg.data = json.dumps(payload, sort_keys=True)
        publisher.publish(msg)
        return True
    except Exception as exc:
        print(f"[ros2] real start status publish failed: {exc}", flush=True)
        return False


__all__ = ["publish_real_start_status"]

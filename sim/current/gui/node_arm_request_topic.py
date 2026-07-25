"""Topic command override path for GUI arm/disarm requests."""

from __future__ import annotations

from .node_command_attempts import should_log_attempt


def publish_arm_override_if_configured(self, value: bool, deadline: float, attempt: int) -> bool:
    if self._arm_mode_command_path not in {"auto", "topic"}:
        return False
    if not self._publish_command_override({"arm": bool(value)}):
        return False
    if should_log_attempt(attempt):
        self._push_event(f"arm command sent via command_override: armed={bool(value)}, attempt={attempt}")
    self._retry_arm_request(value, deadline, attempt)
    return True


__all__ = ["publish_arm_override_if_configured"]

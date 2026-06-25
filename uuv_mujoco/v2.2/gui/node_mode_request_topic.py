"""Topic override path for GUI mode commands."""

from __future__ import annotations

from .node_command_attempts import should_log_attempt


def publish_mode_override_if_configured(self, mode: str, deadline: float, attempt: int) -> bool:
    if self._arm_mode_command_path != "topic":
        return False
    if not self._publish_command_override({"mode": str(mode)}):
        return False
    if should_log_attempt(attempt):
        self._push_event(f"set_mode command sent via command_override: mode={mode}, attempt={attempt}")
    self._retry_mode_request(mode, deadline, attempt)
    return True


__all__ = ["publish_mode_override_if_configured"]

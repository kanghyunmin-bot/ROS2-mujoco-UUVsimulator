"""Command timeout helper for Ros2Bridge public loop methods."""

from __future__ import annotations

import time


def clear_expired_command(self) -> None:
    if self.cmd_active and (time.monotonic() - self.last_cmd_wall > self.cmd_timeout_s):
        self._clear_cmd()


__all__ = ["clear_expired_command"]

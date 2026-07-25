"""Initial-depth-hold vertical feedback state for Ros2Bridge."""

from __future__ import annotations

import time

from .ros2_state_vertical_hold_window import (
    extend_zero_feedback_after_hold_release,
    reset_vertical_feedback_history,
)
from .ros2_state_vertical_zero_reason import sitl_vertical_feedback_zero_reason


def set_sitl_initial_depth_hold_active(self, active: bool) -> None:
    active = bool(active)
    previous = bool(self._sitl_initial_depth_hold_active)
    self._sitl_initial_depth_hold_active = active
    if previous != active:
        reset_vertical_feedback_history(self)
        if not active:
            extend_zero_feedback_after_hold_release(self)


def _sitl_vertical_feedback_zero_reason(self) -> str:
    return sitl_vertical_feedback_zero_reason(self)


def _log_sitl_vertical_feedback_zero(self, reason: str) -> None:
    if not reason:
        return
    now = time.monotonic()
    if now - self._sitl_zero_vertical_feedback_last_log_wall < 2.0:
        return
    self._sitl_zero_vertical_feedback_last_log_wall = now
    print(
        f"[ros2_bridge] SITL vertical velocity feedback zeroed: reason={reason}",
        flush=True,
    )


__all__ = [
    "_log_sitl_vertical_feedback_zero",
    "_sitl_vertical_feedback_zero_reason",
    "set_sitl_initial_depth_hold_active",
]

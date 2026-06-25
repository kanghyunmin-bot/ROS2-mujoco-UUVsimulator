"""Post-release vertical-feedback zeroing windows for Ros2Bridge."""

from __future__ import annotations

import time


def reset_vertical_feedback_history(self) -> None:
    self._sitl_bar30_prev_depth_m = None
    self._sitl_bar30_prev_t = None
    self._sitl_prev_vel_sim_t = None
    self._sitl_prev_vel_enu = None


def extend_zero_feedback_after_hold_release(self) -> None:
    if self._sitl_zero_vertical_feedback_after_hold_release_s <= 0.0:
        return
    self._sitl_zero_vertical_feedback_until_wall = max(
        self._sitl_zero_vertical_feedback_until_wall,
        time.monotonic() + self._sitl_zero_vertical_feedback_after_hold_release_s,
    )


__all__ = ["extend_zero_feedback_after_hold_release", "reset_vertical_feedback_history"]

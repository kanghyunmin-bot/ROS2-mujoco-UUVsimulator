"""Vertical-feedback zeroing reason helpers for Ros2Bridge."""

from __future__ import annotations

import time


def sitl_vertical_feedback_zero_reason(self) -> str:
    if self._sitl_initial_depth_hold_active:
        return "initial_depth_hold"
    if time.monotonic() < self._sitl_zero_vertical_feedback_until_wall:
        return "post_initial_depth_release"
    if (
        self._sitl_zero_vertical_feedback_while_disarmed
        and self._sitl_transport is not None
        and not self._sitl_transport.vehicle_armed
    ):
        return "disarmed"
    return ""


__all__ = ["sitl_vertical_feedback_zero_reason"]

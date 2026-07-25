"""Spin-loop helper for axis RC validation nodes."""

from __future__ import annotations

import time

import rclpy

from axis_rc_node_publish import publish_manual_control, publish_rc_override, publish_rc_release

_MIN_DRAIN_S = 0.0005
_MAX_DRAIN_S = 0.001
_DRAIN_STEP_S = 0.00025


def rc_override_due(input_mode: str, axis: str | None) -> bool:
    return input_mode in ("rc-override", "both") or axis in ("roll", "pitch")


def manual_control_due(input_mode: str) -> bool:
    return input_mode in ("manual-control", "both")


def drain_callbacks(node, budget_s: float) -> None:
    end_t = time.monotonic() + max(0.0, float(budget_s))
    while time.monotonic() < end_t:
        rclpy.spin_once(node, timeout_sec=_DRAIN_STEP_S)


def spin_with_axis_rc(
    node,
    duration: float,
    axis: str | None = None,
    command: float = 0.0,
    hz: float = 100.0,
    input_mode: str = "rc-override",
) -> None:
    end_t = time.monotonic() + float(duration)
    dt = 1.0 / max(1.0, float(hz))
    next_t = time.monotonic()
    while time.monotonic() < end_t:
        loop_start = time.monotonic()
        published = False
        if rc_override_due(input_mode, axis):
            publish_rc_override(node, axis, command)
            published = True
        elif input_mode == "manual-control" and not node._rc_released:
            publish_rc_release(node)
        if manual_control_due(input_mode):
            publish_manual_control(node, axis, command)
            published = True
        if published:
            now = time.monotonic()
            node.current_axis = axis
            node.current_command = float(command)
            node.last_command_publish_wall = now
            node.last_command_publish_t = node.elapsed()
            node.last_command_sequence += 1
            node.last_command_mode = input_mode
        drain_callbacks(node, min(_MAX_DRAIN_S, max(_MIN_DRAIN_S, dt * 0.4)))
        node._sample()
        next_t = max(next_t + dt, loop_start + dt)
        sleep_s = min(next_t - time.monotonic(), end_t - time.monotonic())
        if sleep_s > 0:
            time.sleep(sleep_s)


__all__ = [
    "drain_callbacks",
    "manual_control_due",
    "rc_override_due",
    "spin_with_axis_rc",
]

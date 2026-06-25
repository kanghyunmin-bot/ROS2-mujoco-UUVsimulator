"""RC override spin loop for the roll stability probe."""

from __future__ import annotations

import time

import rclpy


def spin_rc_publish_loop(
    node,
    duration: float,
    *,
    publish_once,
    hz: float = 20.0,
) -> None:
    end_t = time.monotonic() + duration
    dt = 1.0 / hz
    while time.monotonic() < end_t:
        publish_once()
        rclpy.spin_once(node, timeout_sec=min(0.05, dt))
        remaining = end_t - time.monotonic()
        if remaining > 0:
            time.sleep(min(dt, remaining))


__all__ = ["spin_rc_publish_loop"]

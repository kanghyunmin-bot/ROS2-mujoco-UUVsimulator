"""Neutral-control spin helpers for axis RC service calls."""

from __future__ import annotations

import time
from typing import Any

import rclpy


def publish_neutral_and_spin(node: Any, *, timeout_sec: float = 0.05) -> None:
    node.publish_neutral_control()
    rclpy.spin_once(node, timeout_sec=timeout_sec)


def wait_for_future(node: Any, future: Any, deadline: float, *, timeout_sec: float = 0.05) -> None:
    while time.monotonic() < deadline and not future.done():
        publish_neutral_and_spin(node, timeout_sec=timeout_sec)


def spin_neutral_then_sleep(
    node: Any,
    *,
    spin_timeout_sec: float = 0.1,
    sleep_s: float = 0.1,
) -> None:
    publish_neutral_and_spin(node, timeout_sec=spin_timeout_sec)
    time.sleep(float(sleep_s))

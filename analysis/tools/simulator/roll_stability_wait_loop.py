"""Shared wait-loop primitives for roll-stability command probes."""

from __future__ import annotations

import time
from collections.abc import Callable

import rclpy


def deadline_after(timeout: float) -> float:
    return time.monotonic() + float(timeout)


def spin_once(node: object, *, timeout_sec: float = 0.05) -> None:
    rclpy.spin_once(node, timeout_sec=timeout_sec)


def neutral_spin_once(node: object, *, timeout_sec: float = 0.05) -> None:
    node.neutral_rc()
    spin_once(node, timeout_sec=timeout_sec)


def wait_until(deadline: float, poll: Callable[[], bool]) -> bool:
    while time.monotonic() < deadline:
        if poll():
            return True
    return False


__all__ = [
    "deadline_after",
    "neutral_spin_once",
    "spin_once",
    "wait_until",
]

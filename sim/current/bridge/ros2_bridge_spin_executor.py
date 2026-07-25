"""ROS executor spin helpers for Ros2Bridge."""

from __future__ import annotations

import time


def ros_spin_due(self, now_wall: float) -> bool:
    if now_wall + 1.0e-9 < self._ros_next_spin_wall:
        return False
    self._ros_next_spin_wall = now_wall + self._ros_spin_period_s
    return True


def spin_executor_callbacks(self) -> None:
    if self._executor is None:
        return
    for _ in range(self._ros_spin_max_callbacks):
        self._executor.spin_once(timeout_sec=0.0)


def handle_ros_spin_exception(self, exc: Exception) -> None:
    if not self._ros_error_reported:
        self._ros_error_reported = True
        print(f"[ros2_bridge] callbacks disabled: {exc}", flush=True)
    self._ros_ok = False


__all__ = ["handle_ros_spin_exception", "ros_spin_due", "spin_executor_callbacks"]

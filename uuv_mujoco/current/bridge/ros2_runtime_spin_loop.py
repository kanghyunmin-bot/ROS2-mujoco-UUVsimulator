"""Dedicated ROS executor spin loop for Ros2Bridge."""

from __future__ import annotations

from .ros2_runtime_spin_state import (
    mark_spin_error,
    send_cached_external_nav_due,
    spin_context_active,
)


def ros_spin_loop(self) -> None:
    while not self._ros_spin_stop.is_set():
        if not spin_context_active(self):
            break
        try:
            self._executor.spin_once(timeout_sec=self._ros_spin_timeout_s)
            send_cached_external_nav_due(self)
        except Exception as exc:
            if self._is_ros_context_shutdown_error(exc):
                break
            mark_spin_error(self, exc)
            break


__all__ = ["ros_spin_loop"]

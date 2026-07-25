"""`Ros2Bridge.spin_once` implementation."""

from __future__ import annotations

import time

from .ros2_bridge_cmd_timeout import clear_expired_command
from .ros2_bridge_sitl_poll import poll_sitl_servo_if_enabled
from .ros2_bridge_spin_executor import (
    handle_ros_spin_exception,
    ros_spin_due,
    spin_executor_callbacks,
)
from .ros2_rc_override_forward_cache import service_pending_rc_override_forward


def spin_once(self) -> None:
    poll_sitl_servo_if_enabled(self)
    service_pending_rc_override_forward(self)
    if not self._enable_ros or not self._ros_ok:
        clear_expired_command(self)
        return
    if self._ros_executor_spin_thread_enabled:
        clear_expired_command(self)
        return
    now = time.monotonic()
    if not ros_spin_due(self, now):
        return
    try:
        spin_executor_callbacks(self)
    except Exception as exc:
        handle_ros_spin_exception(self, exc)
        return
    clear_expired_command(self)


__all__ = ["spin_once"]

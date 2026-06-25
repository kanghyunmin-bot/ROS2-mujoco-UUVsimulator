"""Shared SITL servo polling helper for Ros2Bridge public loop methods."""

from __future__ import annotations


def poll_sitl_servo_if_enabled(self) -> None:
    if not self.enable_sitl or self._sitl_transport is None:
        return
    with self._sitl_transport_lock:
        self._sitl_transport.poll_servo()


__all__ = ["poll_sitl_servo_if_enabled"]

"""Spin-loop state predicates for Ros2Bridge."""

from __future__ import annotations


def spin_context_active(self) -> bool:
    if not self._enable_ros or not self._ros_ok or self._executor is None:
        return False
    if self._ros_context is None:
        return True
    try:
        return bool(self._ros_context.ok())
    except Exception:
        return False


def send_cached_external_nav_due(self) -> None:
    if self.enable_sitl and self._sitl_transport is not None:
        with self._sitl_transport_lock:
            self._sitl_transport.send_cached_external_nav_due()


def mark_spin_error(self, exc: Exception) -> None:
    if not self._ros_error_reported:
        self._ros_error_reported = True
        print(f"[ros2_bridge] executor spin thread stopped: {exc}", flush=True)
    self._ros_ok = False


__all__ = ["mark_spin_error", "send_cached_external_nav_due", "spin_context_active"]

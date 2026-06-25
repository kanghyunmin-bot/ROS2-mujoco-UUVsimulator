"""Timestamp acquisition for Ros2Bridge publish path."""

from __future__ import annotations


def acquire_ros_stamp(self):
    try:
        return self.node.get_clock().now().to_msg()
    except Exception as exc:
        if not self._ros_error_reported:
            self._ros_error_reported = True
            print(f"[ros2_bridge] timestamp acquisition failed: {exc}", flush=True)
        self._ros_ok = False
        return None


__all__ = ["acquire_ros_stamp"]

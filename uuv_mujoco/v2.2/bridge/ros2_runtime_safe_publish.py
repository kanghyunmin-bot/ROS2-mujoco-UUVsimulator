"""Safe ROS publisher wrapper for Ros2Bridge."""

from __future__ import annotations


def safe_publish(self, publisher, msg, label: str) -> bool:
    if publisher is None:
        return True
    try:
        publisher.publish(msg)
        return True
    except Exception as exc:
        if not self._ros_error_reported:
            self._ros_error_reported = True
            print(f"[ros2_bridge] publish blocked ({label}): {exc}", flush=True)
        self._ros_ok = False
        return False


__all__ = ["safe_publish"]

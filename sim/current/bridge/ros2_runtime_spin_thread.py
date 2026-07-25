"""Dedicated ROS executor thread startup for Ros2Bridge."""

from __future__ import annotations

import threading


def start_ros_spin_thread(self) -> None:
    if self._executor is None or self._ros_spin_thread is not None:
        return
    self._ros_spin_stop.clear()
    self._ros_spin_thread = threading.Thread(
        target=self._ros_spin_loop,
        name="uuv_ros2_executor",
        daemon=True,
    )
    self._ros_spin_thread.start()
    print(
        "[ros2_bridge] dedicated executor spin thread enabled "
        f"(timeout={self._ros_spin_timeout_s:.3f}s)",
        flush=True,
    )


__all__ = ["start_ros_spin_thread"]

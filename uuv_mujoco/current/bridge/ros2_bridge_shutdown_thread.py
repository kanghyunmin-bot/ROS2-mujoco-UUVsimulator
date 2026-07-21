"""ROS spin-thread shutdown helper."""

from __future__ import annotations

import threading


def stop_ros_spin_thread(self) -> None:
    self._ros_spin_stop.set()
    spin_thread = self._ros_spin_thread
    if spin_thread is not None and spin_thread.is_alive() and spin_thread is not threading.current_thread():
        try:
            spin_thread.join(timeout=1.0)
        except Exception:
            pass
    self._ros_spin_thread = None


def stop_sitl_poll_thread(self) -> None:
    stop_poll = getattr(self, "_stop_sitl_poll_thread", None)
    if callable(stop_poll):
        stop_poll()


__all__ = ["stop_ros_spin_thread", "stop_sitl_poll_thread"]

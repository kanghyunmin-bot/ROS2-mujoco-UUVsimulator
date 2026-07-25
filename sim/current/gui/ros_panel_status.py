"""Thread-safe status updates for the GUI ROS panel."""

from __future__ import annotations

import threading


def set_tk_status(root, variable, text: str) -> None:
    if threading.current_thread() is threading.main_thread():
        variable.set(text)
        return
    try:
        root.after(0, lambda: variable.set(text))
    except Exception:
        pass


def _set_ros_pkg_status(self, text: str) -> None:
    set_tk_status(self.root, self.ros_pkg_status_var, text)


def _set_rviz_status(self, text: str) -> None:
    set_tk_status(self.root, self.rviz_status_var, text)


__all__ = ["_set_ros_pkg_status", "_set_rviz_status", "set_tk_status"]

"""Thread-slot mapping for GUI-started ROS processes."""

from __future__ import annotations

from typing import Any


THREAD_ATTR_BY_PROCESS_ATTR = {
    "_ros_pkg_process": "_ros_pkg_thread",
    "_ros_build_process": "_ros_build_thread",
    "_rviz_process": "_rviz_thread",
}


def bind_logged_process_thread(owner: Any, attr_name: str, thread: Any) -> None:
    thread_attr = THREAD_ATTR_BY_PROCESS_ATTR.get(attr_name)
    if thread_attr is not None:
        setattr(owner, thread_attr, thread)


__all__ = ["THREAD_ATTR_BY_PROCESS_ATTR", "bind_logged_process_thread"]

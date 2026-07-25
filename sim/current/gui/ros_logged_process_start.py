"""Start-sequence orchestration for GUI-started ROS processes."""

from __future__ import annotations

import threading
from typing import Any, Callable

from .ros_logged_process_files import open_logged_process_file
from .ros_logged_process_launcher import launch_logged_ros_process
from .ros_logged_process_threads import bind_logged_process_thread


def start_logged_ros_process(
    owner: Any,
    *,
    cmd: list[str],
    label: str,
    log_prefix: str,
    attr_name: str,
    status_callback: Callable[[str], None],
) -> None:
    opened_log = open_logged_process_file(
        label=label,
        log_prefix=log_prefix,
        status_callback=status_callback,
    )
    if opened_log is None:
        return
    log_path, log_file = opened_log
    proc = launch_logged_ros_process(
        cmd=cmd,
        label=label,
        log_file=log_file,
        status_callback=status_callback,
    )
    if proc is None:
        return

    setattr(owner, attr_name, proc)
    status_callback(f"{label}: starting ({log_path.name})")
    owner.node.push_event(f"{label} start requested: {log_path.name}")
    thread = threading.Thread(
        target=owner._watch_ros_process,
        args=(proc, log_path, label, attr_name, status_callback),
        daemon=True,
    )
    bind_logged_process_thread(owner, attr_name, thread)
    thread.start()
    owner._refresh_ros2_buttons()


__all__ = ["start_logged_ros_process"]

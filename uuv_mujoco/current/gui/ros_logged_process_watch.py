"""Watcher orchestration for GUI-started ROS processes."""

from __future__ import annotations

from pathlib import Path
import subprocess
from typing import Any, Callable

from .ros_logged_process_finish import schedule_logged_process_finish
from .ros_logged_process_tail import read_logged_process_until_exit


def watch_ros_process(
    owner: Any,
    proc: subprocess.Popen[str],
    log_path: Path,
    label: str,
    attr_name: str,
    status_callback: Callable[[str], None],
) -> None:
    return_code, last_line = read_logged_process_until_exit(owner, proc, log_path, label)
    schedule_logged_process_finish(
        owner,
        proc=proc,
        label=label,
        attr_name=attr_name,
        status_callback=status_callback,
        return_code=return_code,
        last_line=last_line,
    )


__all__ = ["watch_ros_process"]

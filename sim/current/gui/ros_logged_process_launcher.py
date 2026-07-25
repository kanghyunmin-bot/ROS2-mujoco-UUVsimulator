"""Subprocess launch helper for GUI-started ROS processes."""

from __future__ import annotations

import subprocess
from typing import Callable, TextIO

from .config import ROS_WORKSPACE_DIR


def launch_logged_ros_process(
    *,
    cmd: list[str],
    label: str,
    log_file: TextIO,
    status_callback: Callable[[str], None],
) -> subprocess.Popen[str] | None:
    try:
        return subprocess.Popen(
            cmd,
            cwd=str(ROS_WORKSPACE_DIR),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
    except Exception as exc:
        status_callback(f"{label}: start failed: {exc}")
        return None
    finally:
        try:
            log_file.close()
        except Exception:
            pass


__all__ = ["launch_logged_ros_process"]

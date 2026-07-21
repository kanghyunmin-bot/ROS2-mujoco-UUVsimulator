"""Log tailing for GUI-started ROS processes."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .ros_log_events import LOG_EVENT_PREFIXES
from .ros_log_tail_loop import read_remaining_log_lines, tail_active_log


def read_logged_process_until_exit(owner: Any, proc: Any, log_path: Path, label: str) -> tuple[int, str]:
    last_line = ""
    try:
        with log_path.open("r", encoding="utf-8", errors="replace") as log_stream:
            last_line = tail_active_log(owner, proc, log_stream, label)
            last_line = read_remaining_log_lines(log_stream, last_line)
        return_code = proc.returncode if proc.returncode is not None else proc.wait()
    except Exception as exc:
        return -1, f"reader failed: {exc}"
    return int(return_code), last_line


__all__ = ["LOG_EVENT_PREFIXES", "read_logged_process_until_exit"]

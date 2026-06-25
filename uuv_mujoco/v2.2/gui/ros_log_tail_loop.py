"""Active and remaining log-read loops for GUI-started ROS processes."""

from __future__ import annotations

import time
from typing import Any

from .ros_log_events import maybe_push_log_event


def tail_active_log(owner: Any, proc: Any, log_stream: Any, label: str) -> str:
    last_line = ""
    while True:
        raw_line = log_stream.readline()
        if raw_line:
            line = raw_line.strip()
            if not line:
                continue
            last_line = line
            maybe_push_log_event(owner, label, line)
            continue
        if proc.poll() is not None:
            break
        time.sleep(0.1)
    return last_line


def read_remaining_log_lines(log_stream: Any, last_line: str) -> str:
    for raw_line in log_stream:
        line = raw_line.strip()
        if line:
            last_line = line
    return last_line


__all__ = ["read_remaining_log_lines", "tail_active_log"]

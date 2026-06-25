"""GUI event extraction from ROS process log lines."""

from __future__ import annotations

from typing import Any


LOG_EVENT_PREFIXES = ("[INFO]", "[WARN]", "[ERROR]", "[rviz", "[ros2", "[colcon")
MAX_EVENT_LINE_LEN = 150


def maybe_push_log_event(owner: Any, label: str, line: str) -> None:
    if not line.startswith(LOG_EVENT_PREFIXES):
        return
    owner.node.push_event(f"{label}: {short_log_line(line)}")


def short_log_line(line: str) -> str:
    return line if len(line) <= MAX_EVENT_LINE_LEN else f"{line[:MAX_EVENT_LINE_LEN - 3]}..."


__all__ = ["LOG_EVENT_PREFIXES", "MAX_EVENT_LINE_LEN", "maybe_push_log_event", "short_log_line"]

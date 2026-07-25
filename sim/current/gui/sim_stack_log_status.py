"""Status-line parsing for GUI-owned simulator stack logs."""

from __future__ import annotations


STATUS_PREFIXES = (
    "[start]",
    "[reset]",
    "[launch]",
    "[runtime]",
    "[physics]",
    "[sitl]",
    "[model]",
)

START_STATUS_PREFIXES = ("[start]", "[reset]")
INIT_STATE_READY_PREFIX = "[runtime] real start state OK"
HEADLESS_MODE_PREFIX = "[runtime] headless mode enabled"


def compact_status_line(line: str, *, limit: int = 150) -> str:
    if len(line) <= limit:
        return line
    return f"{line[: limit - 3]}..."


def is_status_line(line: str) -> bool:
    return line.startswith(STATUS_PREFIXES)


def status_text_for_line(line: str, short_line: str, current_status: str) -> str | None:
    if line.startswith(START_STATUS_PREFIXES):
        return f"sim: {short_line}"
    if line.startswith(INIT_STATE_READY_PREFIX):
        return "sim: init state OK; waiting for COMMAND READY"
    if line.startswith(HEADLESS_MODE_PREFIX) and current_status.startswith("sim: starting"):
        return "sim: running; waiting for READY"
    return None


__all__ = [
    "STATUS_PREFIXES",
    "compact_status_line",
    "is_status_line",
    "status_text_for_line",
]

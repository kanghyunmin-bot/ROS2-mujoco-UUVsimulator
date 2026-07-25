"""Line handling for GUI-owned simulator stack logs."""

from __future__ import annotations

from .sim_stack_log_status import compact_status_line, is_status_line, status_text_for_line


def handle_sim_stack_log_line(owner, raw_line: str, last_line: str) -> str:
    line = raw_line.strip()
    if not line:
        return last_line
    if not is_status_line(line):
        return line

    short_line = compact_status_line(line)
    owner.node.push_event(short_line)
    status_text = status_text_for_line(line, short_line, owner.sim_stack_status_var.get())
    if status_text is not None:
        owner._set_sim_stack_status(status_text)
    return line


__all__ = ["handle_sim_stack_log_line"]

"""Finish-state handling for GUI-started ROS processes."""

from __future__ import annotations

from typing import Any, Callable


def schedule_logged_process_finish(
    owner: Any,
    *,
    proc: Any,
    label: str,
    attr_name: str,
    status_callback: Callable[[str], None],
    return_code: int,
    last_line: str,
) -> None:
    def finish() -> None:
        finish_logged_process(
            owner,
            proc=proc,
            label=label,
            attr_name=attr_name,
            status_callback=status_callback,
            return_code=return_code,
            last_line=last_line,
        )

    try:
        owner.root.after(0, finish)
    except Exception:
        pass


def finish_logged_process(
    owner: Any,
    *,
    proc: Any,
    label: str,
    attr_name: str,
    status_callback: Callable[[str], None],
    return_code: int,
    last_line: str,
) -> None:
    if getattr(owner, attr_name) is proc:
        setattr(owner, attr_name, None)
    if return_code == 0:
        _report_exit(owner, label, status_callback)
    elif return_code < 0:
        _report_stop(owner, label, status_callback)
    else:
        _report_failure(owner, label, status_callback, return_code, last_line)
    owner._refresh_ros2_buttons()


def _report_exit(owner: Any, label: str, status_callback: Callable[[str], None]) -> None:
    status_callback(f"{label}: exited")
    owner.node.push_event(f"{label} exited")


def _report_stop(owner: Any, label: str, status_callback: Callable[[str], None]) -> None:
    status_callback(f"{label}: stopped")
    owner.node.push_event(f"{label} stopped")


def _report_failure(
    owner: Any,
    label: str,
    status_callback: Callable[[str], None],
    return_code: int,
    last_line: str,
) -> None:
    text = last_line if last_line else f"rc={return_code}"
    status_callback(f"{label} failed: {text}")
    owner.node.push_event(f"{label} failed: {text}")


__all__ = ["finish_logged_process", "schedule_logged_process_finish"]

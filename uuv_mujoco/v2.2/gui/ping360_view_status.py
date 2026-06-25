"""Thread-safe Ping360 viewer status updates."""

from __future__ import annotations

import threading


def ping360_view_running(process) -> bool:
    return process is not None and process.poll() is None


def set_ping360_view_status(owner, text: str) -> None:
    if threading.current_thread() is threading.main_thread():
        owner.ping360_view_status_var.set(text)
        return
    try:
        owner.root.after(0, lambda: owner.ping360_view_status_var.set(text))
    except Exception:
        pass


__all__ = ["ping360_view_running", "set_ping360_view_status"]

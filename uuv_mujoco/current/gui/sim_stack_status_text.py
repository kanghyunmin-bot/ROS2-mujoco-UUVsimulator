"""Thread-safe simulator-stack status text updates."""

from __future__ import annotations

import threading
from typing import Any


def set_sim_stack_status_text(root: Any, status_var: Any, text: str) -> None:
    if threading.current_thread() is threading.main_thread():
        status_var.set(text)
        return
    try:
        root.after(0, lambda: status_var.set(text))
    except Exception:
        pass


__all__ = ["set_sim_stack_status_text"]

"""Reset-thread launcher for the simulator stack."""

from __future__ import annotations

import threading


def reset_thread_running(thread: threading.Thread | None) -> bool:
    return thread is not None and thread.is_alive()


def start_reset_thread(owner, target) -> None:
    if reset_thread_running(owner._sim_stack_reset_thread):
        return
    owner._sim_stack_reset_thread = threading.Thread(target=target, daemon=True)
    owner._sim_stack_reset_thread.start()


__all__ = ["reset_thread_running", "start_reset_thread"]

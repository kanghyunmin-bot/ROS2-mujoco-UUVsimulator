"""Pure status/control policy for GUI simulator-stack state."""

from __future__ import annotations

from threading import Thread


SIM_STACK_STATUS_PROBE_INTERVAL_S = 1.0


def reset_thread_running(reset_thread: Thread | None) -> bool:
    return reset_thread is not None and reset_thread.is_alive()


def sim_stack_controls_running(
    *,
    tracked: bool,
    external_cached: bool,
    reset_running: bool,
) -> bool:
    return bool(tracked or external_cached or reset_running)


def next_sim_stack_status(
    *,
    current: str,
    tracked: bool,
    external: bool,
) -> str | None:
    if tracked:
        if current.startswith(("sim: stopped", "sim: exited", "sim failed")):
            return "sim: running"
        return None
    if external:
        if not current.startswith("sim: running (external)"):
            return "sim: running (external)"
        return None
    if current.startswith(("sim: running", "sim: backend running")):
        return "sim: stopped"
    return None


__all__ = [
    "SIM_STACK_STATUS_PROBE_INTERVAL_S",
    "next_sim_stack_status",
    "reset_thread_running",
    "sim_stack_controls_running",
]

"""Refresh orchestration for GUI simulator-stack status."""

from __future__ import annotations

import time
from typing import Any

from .sim_stack_status_logic import (
    SIM_STACK_STATUS_PROBE_INTERVAL_S,
    next_sim_stack_status,
    reset_thread_running,
)


def refresh_sim_stack_status(owner: Any) -> None:
    now = time.monotonic()
    last_probe = getattr(owner, "_last_sim_stack_probe_wall", -1.0)
    if last_probe >= 0.0 and now - last_probe < SIM_STACK_STATUS_PROBE_INTERVAL_S:
        owner._refresh_sim_stack_controls()
        return
    owner._last_sim_stack_probe_wall = now

    if reset_thread_running(owner._sim_stack_reset_thread):
        owner._refresh_sim_stack_controls()
        return

    tracked = owner._tracked_sim_stack_running()
    external = owner._external_sim_stack_running()
    owner._external_sim_stack_running_cached = external
    next_status = next_sim_stack_status(
        current=owner.sim_stack_status_var.get(),
        tracked=tracked,
        external=external,
    )
    if next_status:
        owner._set_sim_stack_status(next_status)
    owner._refresh_sim_stack_controls()


__all__ = ["refresh_sim_stack_status"]

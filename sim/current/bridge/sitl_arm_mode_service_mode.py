"""Pending mode retry service loop for SitlTransport."""

from __future__ import annotations

from .sitl_arm_mode_service_send import send_pending_mode_to_any_link
from .sitl_arm_mode_service_state import (
    pending_mode_reached,
    pending_mode_send_due,
    pending_mode_text,
    pending_mode_timed_out,
)


def service_pending_mode_command(transport, now_wall: float) -> None:
    mode = pending_mode_text(transport)
    if not mode:
        return
    if pending_mode_reached(transport, mode):
        print(f"[sitl_transport] pending mode target reached: mode={mode!r}", flush=True)
        transport._sitl_pending_mode = ""
        return
    if pending_mode_timed_out(transport, now_wall):
        print(f"[sitl_transport] pending mode target timed out: mode={mode!r}", flush=True)
        transport._sitl_pending_mode = ""
        return
    if not pending_mode_send_due(transport, now_wall):
        return
    if send_pending_mode_to_any_link(transport, mode):
        transport._sitl_pending_mode_last_send_wall = now_wall


__all__ = ["service_pending_mode_command"]

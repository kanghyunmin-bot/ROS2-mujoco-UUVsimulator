"""Pending arm retry service loop for SitlTransport."""

from __future__ import annotations

from .sitl_arm_mode_service_send import send_pending_arm_to_any_link
from .sitl_arm_mode_service_state import (
    pending_arm_reached,
    pending_arm_send_due,
    pending_arm_target,
    pending_arm_timed_out,
)


def service_pending_arm_command(transport, now_wall: float) -> None:
    target_arm = pending_arm_target(transport)
    if target_arm is None:
        return
    reached_after_wall = float(getattr(transport, "_sitl_pending_arm_reached_after_wall", -1.0))
    if reached_after_wall > 0.0 and now_wall < reached_after_wall:
        if pending_arm_send_due(transport, now_wall) and send_pending_arm_to_any_link(transport, target_arm):
            transport._sitl_pending_arm_last_send_wall = now_wall
            if transport._sitl_cmd_debug:
                print(f"[sitl_transport] pending arm command resent: armed={target_arm}", flush=True)
        return
    if pending_arm_reached(transport, target_arm):
        print(f"[sitl_transport] pending arm target reached: armed={target_arm}", flush=True)
        transport._sitl_pending_arm_target = None
        transport._sitl_pending_arm_reached_after_wall = -1.0
        return
    if pending_arm_timed_out(transport, now_wall):
        print(f"[sitl_transport] pending arm target timed out: armed={target_arm}", flush=True)
        transport._sitl_pending_arm_target = None
        transport._sitl_pending_arm_reached_after_wall = -1.0
        return
    if not pending_arm_send_due(transport, now_wall):
        return
    if send_pending_arm_to_any_link(transport, target_arm):
        transport._sitl_pending_arm_last_send_wall = now_wall
        if transport._sitl_cmd_debug:
            print(f"[sitl_transport] pending arm command resent: armed={target_arm}", flush=True)


__all__ = ["service_pending_arm_command"]

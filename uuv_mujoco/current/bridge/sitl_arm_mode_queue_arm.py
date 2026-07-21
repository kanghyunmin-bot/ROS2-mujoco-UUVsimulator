"""Arm queue helper for SitlTransport."""

from __future__ import annotations

import time

from .sitl_arm_mode_service_state import ARM_OPPOSITE_COMMAND_GRACE_S


def queue_arm_command_impl(transport, arm: bool) -> bool:
    now_wall = time.monotonic()
    target_arm = bool(arm)
    recent_opposite_command = _recent_opposite_arm_command(transport, target_arm, now_wall)
    had_opposite_pending = (
        transport._sitl_pending_arm_target is not None
        and bool(transport._sitl_pending_arm_target) != target_arm
    )
    hold_for_opposite_command = bool(had_opposite_pending or recent_opposite_command)
    if (
        transport._sitl_pending_arm_target is not None
        and bool(transport._sitl_pending_arm_target) == target_arm
    ):
        transport._service_pending_arm_command(now_wall)
        return True
    if (
        bool(transport._sitl_vehicle_armed) == target_arm
        and transport._sitl_pending_arm_target is None
        and not recent_opposite_command
    ):
        transport._sitl_last_arm_command_target = target_arm
        transport._sitl_last_arm_command_wall = now_wall
        return True
    transport._sitl_pending_arm_target = target_arm
    transport._sitl_pending_arm_start_wall = now_wall
    transport._sitl_pending_arm_last_send_wall = 0.0
    transport._sitl_pending_arm_reached_after_wall = (
        now_wall + ARM_OPPOSITE_COMMAND_GRACE_S if hold_for_opposite_command else -1.0
    )
    transport._sitl_pending_arm_neutral_sent = False
    transport._sitl_last_arm_command_target = target_arm
    transport._sitl_last_arm_command_wall = now_wall
    print(f"[sitl_transport] arm/disarm command queued: armed={target_arm}", flush=True)
    transport._service_pending_arm_command(now_wall)
    return True


def _recent_opposite_arm_command(transport, target_arm: bool, now_wall: float) -> bool:
    last_target = getattr(transport, "_sitl_last_arm_command_target", None)
    if last_target is None or bool(last_target) == bool(target_arm):
        return False
    last_wall = float(getattr(transport, "_sitl_last_arm_command_wall", -1.0))
    return last_wall > 0.0 and now_wall - last_wall < ARM_OPPOSITE_COMMAND_GRACE_S


__all__ = ["queue_arm_command_impl"]

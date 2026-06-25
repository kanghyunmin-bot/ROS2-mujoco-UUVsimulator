"""Arm queue helper for SitlTransport."""

from __future__ import annotations

import time


def queue_arm_command_impl(transport, arm: bool) -> bool:
    now_wall = time.monotonic()
    target_arm = bool(arm)
    had_opposite_pending = (
        transport._sitl_pending_arm_target is not None
        and bool(transport._sitl_pending_arm_target) != target_arm
    )
    if (
        transport._sitl_pending_arm_target is not None
        and bool(transport._sitl_pending_arm_target) == target_arm
    ):
        transport._service_pending_arm_command(now_wall)
        return True
    if bool(transport._sitl_vehicle_armed) == target_arm and transport._sitl_pending_arm_target is None:
        return True
    transport._sitl_pending_arm_target = target_arm
    transport._sitl_pending_arm_start_wall = now_wall
    transport._sitl_pending_arm_last_send_wall = 0.0
    transport._sitl_pending_arm_reached_after_wall = now_wall + 2.0 if had_opposite_pending else -1.0
    print(f"[sitl_transport] arm/disarm command queued: armed={target_arm}", flush=True)
    transport._service_pending_arm_command(now_wall)
    return True


__all__ = ["queue_arm_command_impl"]

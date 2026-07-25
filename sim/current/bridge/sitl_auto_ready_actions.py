"""Arm/mode actions for SITL auto-ready sequencing."""

from __future__ import annotations


def request_auto_ready_arm_if_needed(self, now_wall: float) -> bool:
    if self._sitl_vehicle_armed:
        return False
    self._set_auto_ready_state("arming", now_wall)
    self._send_auto_ready_neutral_rc(now_wall)
    self.queue_arm_command(True)
    return True


def request_auto_ready_mode_if_needed(self, now_wall: float, target_mode: str) -> bool:
    if str(self._sitl_vehicle_mode or "").upper() == target_mode:
        return False
    self._set_auto_ready_state(f"setting_mode_{target_mode}", now_wall)
    self._send_auto_ready_neutral_rc(now_wall)
    self.queue_set_mode(target_mode)
    return True


def mark_auto_ready_finished(self, now_wall: float) -> None:
    self._sitl_auto_ready_done_wall = now_wall
    self._set_auto_ready_state("ready", now_wall)


__all__ = [
    "mark_auto_ready_finished",
    "request_auto_ready_arm_if_needed",
    "request_auto_ready_mode_if_needed",
]

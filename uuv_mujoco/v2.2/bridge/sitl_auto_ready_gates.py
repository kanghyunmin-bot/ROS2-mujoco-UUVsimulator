"""Readiness gates for SITL auto-ready sequencing."""

from __future__ import annotations

from .sitl_auto_ready_messages import auto_ready_started_message


def auto_ready_target_mode(self) -> str:
    return str(self._sitl_auto_ready_mode or "MANUAL").upper()


def auto_ready_is_disabled(self) -> bool:
    return (not self._sitl_auto_ready_enabled) or bool(self._plant_replay_mode)


def auto_ready_done_still_valid(self, target_mode: str) -> bool:
    del target_mode
    if self._sitl_auto_ready_done_wall <= 0.0:
        return False
    # Auto-ready is a startup bootstrap. Once it has armed the vehicle, operator
    # mode changes must not cause the bootstrap to reassert its initial mode.
    if self._sitl_vehicle_armed:
        return True
    self._sitl_auto_ready_done_wall = -1.0
    return False


def auto_ready_wait_state(self, now_wall: float) -> str | None:
    if not self.rc_override_ready:
        return "waiting_mavlink"
    if not self._auto_ready_extnav_ready(now_wall):
        return "waiting_extnav"
    return None


def mark_auto_ready_started_if_needed(self, now_wall: float, target_mode: str) -> None:
    if self._sitl_auto_ready_started_wall > 0.0:
        return
    self._sitl_auto_ready_started_wall = now_wall
    print(auto_ready_started_message(target_mode), flush=True)


__all__ = [
    "auto_ready_done_still_valid",
    "auto_ready_is_disabled",
    "auto_ready_target_mode",
    "auto_ready_wait_state",
    "mark_auto_ready_started_if_needed",
]

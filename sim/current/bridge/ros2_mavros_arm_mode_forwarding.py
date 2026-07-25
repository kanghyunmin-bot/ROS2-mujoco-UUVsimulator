"""Bridge-level forwarding policy for MAVROS arm/mode requests."""

from __future__ import annotations

from .ros2_mavros_arm_mode_guard import (
    reject_arm_during_boot_guard,
    reject_arm_mode_forwarding_disabled,
    reject_mode_during_boot_guard,
)
from .ros2_mavros_arm_mode_transport import send_arm_to_sitl, send_mode_to_sitl


def _forward_arm_request(self, arm_value: bool, source: str) -> bool:
    arm_value = bool(arm_value)
    if reject_arm_during_boot_guard(self, arm_value, source):
        return False
    if reject_arm_mode_forwarding_disabled(self, source, "arm", arm_value):
        return False
    forward_ok = send_arm_to_sitl(self, arm_value)
    if forward_ok:
        self._mavros_armed = arm_value
    return bool(forward_ok)


def _forward_mode_request(self, mode: str, source: str) -> bool:
    mode = str(mode or "").strip()
    if reject_mode_during_boot_guard(self, mode, source):
        return False
    if reject_arm_mode_forwarding_disabled(self, source, "mode", mode):
        return False
    forward_ok = send_mode_to_sitl(self, mode)
    if forward_ok and mode:
        self._mavros_mode = mode
    return bool(forward_ok)


__all__ = ["_forward_arm_request", "_forward_mode_request"]

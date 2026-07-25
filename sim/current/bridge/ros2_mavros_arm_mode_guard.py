"""Boot-guard and forwarding-enable checks for MAVROS arm/mode requests."""

from __future__ import annotations

import time


def arm_mode_boot_guard_left_s(self) -> float:
    return float(self._arm_mode_boot_guard_s - (time.monotonic() - self._startup_wall))


def reject_arm_during_boot_guard(self, arm_value: bool, source: str) -> bool:
    guard_left_s = arm_mode_boot_guard_left_s(self)
    if bool(arm_value) and guard_left_s > 0.0:
        print(
            f"[bridge] {source} arm ignored during boot guard "
            f"({guard_left_s:.1f}s left, requested armed=True)",
            flush=True,
        )
        return True
    return False


def reject_mode_during_boot_guard(self, mode: str, source: str) -> bool:
    guard_left_s = arm_mode_boot_guard_left_s(self)
    if mode and mode.upper() not in {"", "MANUAL"} and guard_left_s > 0.0:
        print(
            f"[bridge] {source} mode ignored during boot guard "
            f"({guard_left_s:.1f}s left, requested mode={mode!r})",
            flush=True,
        )
        return True
    return False


def reject_arm_mode_forwarding_disabled(self, source: str, request_kind: str, request_value) -> bool:
    if self._mavros_forward_arm_mode:
        return False
    if request_kind == "arm":
        print(
            f"[bridge] {source} arm ignored by ROS2_UUV_MAVROS_FORWARD_ARM_MODE=0 "
            f"(requested armed={request_value})",
            flush=True,
        )
    else:
        print(
            f"[bridge] {source} mode ignored by ROS2_UUV_MAVROS_FORWARD_ARM_MODE=0 "
            f"(requested mode={request_value!r})",
            flush=True,
        )
    return True


__all__ = [
    "arm_mode_boot_guard_left_s",
    "reject_arm_during_boot_guard",
    "reject_arm_mode_forwarding_disabled",
    "reject_mode_during_boot_guard",
]

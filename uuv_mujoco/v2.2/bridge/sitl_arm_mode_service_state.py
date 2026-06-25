"""State predicates for pending SitlTransport arm/mode retries."""

from __future__ import annotations

ARM_PENDING_TIMEOUT_S = 90.0
ARM_PENDING_RESEND_PERIOD_S = 0.02
MODE_PENDING_TIMEOUT_S = 30.0
MODE_PENDING_RESEND_PERIOD_S = 0.02


def pending_arm_target(self) -> bool | None:
    target_arm = self._sitl_pending_arm_target
    return None if target_arm is None else bool(target_arm)


def pending_mode_text(self) -> str:
    return str(self._sitl_pending_mode or "").strip()


def pending_arm_reached(self, target_arm: bool) -> bool:
    return bool(self._sitl_vehicle_armed) == bool(target_arm)


def pending_mode_reached(self, mode: str) -> bool:
    return str(self._sitl_vehicle_mode or "").upper() == str(mode).upper()


def pending_arm_timed_out(self, now_wall: float) -> bool:
    return now_wall - self._sitl_pending_arm_start_wall > ARM_PENDING_TIMEOUT_S


def pending_mode_timed_out(self, now_wall: float) -> bool:
    return now_wall - self._sitl_pending_mode_start_wall > MODE_PENDING_TIMEOUT_S


def pending_arm_send_due(self, now_wall: float) -> bool:
    return now_wall - self._sitl_pending_arm_last_send_wall >= ARM_PENDING_RESEND_PERIOD_S


def pending_mode_send_due(self, now_wall: float) -> bool:
    return now_wall - self._sitl_pending_mode_last_send_wall >= MODE_PENDING_RESEND_PERIOD_S


__all__ = [
    "ARM_PENDING_RESEND_PERIOD_S",
    "ARM_PENDING_TIMEOUT_S",
    "MODE_PENDING_RESEND_PERIOD_S",
    "MODE_PENDING_TIMEOUT_S",
    "pending_arm_reached",
    "pending_arm_send_due",
    "pending_arm_target",
    "pending_arm_timed_out",
    "pending_mode_reached",
    "pending_mode_send_due",
    "pending_mode_text",
    "pending_mode_timed_out",
]

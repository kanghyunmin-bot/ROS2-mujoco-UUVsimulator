"""Arm/mode command retry scheduling helpers."""

from __future__ import annotations

from .runtime import time


def _arm_request_current(self, value: bool) -> bool:
    latest_target = getattr(self, "_latest_arm_target", None)
    return latest_target is None or bool(value) == bool(latest_target)


def _mode_request_current(self, mode: str) -> bool:
    latest_target = str(getattr(self, "_latest_mode_target", "") or "").strip().upper()
    return not latest_target or str(mode).strip().upper() == latest_target


def _retry_arm_request(self, value: bool, deadline: float, attempt: int) -> None:
    if not _arm_request_current(self, value):
        return
    if time.monotonic() >= deadline:
        self._push_event(f"arm target timeout: armed={value}")
        return
    self._schedule_once(
        self._control_request_retry_s,
        lambda: self._send_arm_request(value, deadline, attempt + 1)
        if _arm_request_current(self, value)
        else None,
    )


def _retry_mode_request(self, mode: str, deadline: float, attempt: int) -> None:
    if not _mode_request_current(self, mode):
        return
    if time.monotonic() >= deadline:
        self._push_event(f"set_mode target timeout: {mode}")
        return
    self._schedule_once(
        self._control_request_retry_s,
        lambda: self._send_mode_request(mode, deadline, attempt + 1)
        if _mode_request_current(self, mode)
        else None,
    )


__all__ = ["_retry_arm_request", "_retry_mode_request"]

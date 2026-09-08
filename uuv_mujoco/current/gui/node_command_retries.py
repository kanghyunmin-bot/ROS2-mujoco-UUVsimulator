"""Arm/mode command retry scheduling helpers."""

from __future__ import annotations

from .node_arm_rc_sequence import (
    arm_command_request_current,
    cancel_arm_rc_sequence,
    current_arm_command_generation,
    finish_arm_command_generation,
)
from .runtime import time


def _arm_request_current(
    self,
    value: bool,
    request_generation: int | None = None,
) -> bool:
    return arm_command_request_current(self, value, request_generation)


def _mode_request_current(self, mode: str) -> bool:
    latest_target = str(getattr(self, "_latest_mode_target", "") or "").strip().upper()
    return not latest_target or str(mode).strip().upper() == latest_target


def _retry_arm_request(
    self,
    value: bool,
    deadline: float,
    attempt: int,
    *,
    request_generation: int | None = None,
) -> None:
    if request_generation is None:
        request_generation = current_arm_command_generation(self)
    with self._arm_rc_sequence_lock:
        if not _arm_request_current(self, value, request_generation):
            return
        _retry_current_arm_request(
            self,
            value,
            deadline,
            attempt,
            int(request_generation),
        )


def _retry_current_arm_request(
    self,
    value: bool,
    deadline: float,
    attempt: int,
    request_generation: int,
) -> None:
    if time.monotonic() >= deadline:
        finish_arm_command_generation(self, request_generation)
        cancel_arm_rc_sequence(
            self,
            f"arm target timeout: armed={bool(value)}",
            force_neutral=True,
        )
        self._push_event(f"arm target timeout: armed={value}")
        return
    self._schedule_once(
        self._control_request_retry_s,
        lambda: self._send_arm_request(
            value,
            deadline,
            attempt + 1,
            request_generation=request_generation,
        )
        if _arm_request_current(self, value, request_generation)
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

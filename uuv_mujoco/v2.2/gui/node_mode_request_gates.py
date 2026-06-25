"""Gate checks for GUI mode command requests."""

from __future__ import annotations

import time

from .node_command_attempts import should_log_attempt


def handle_alt_hold_initial_depth_gate(self, mode: str, deadline: float, attempt: int) -> bool:
    del mode, deadline, attempt
    return False


def handle_mode_gate(self, mode: str, deadline: float, attempt: int) -> bool:
    gate_reason = self._arm_mode_gate_reason(mode=mode)
    if not gate_reason:
        return False
    if time.monotonic() >= deadline:
        self._push_event(f"set_mode {mode} blocked: {gate_reason}")
        return True
    if should_log_attempt(attempt):
        self._push_event(f"set_mode {mode} delayed: {gate_reason}")
    self._retry_mode_request(mode, deadline, attempt)
    return True


__all__ = ["handle_alt_hold_initial_depth_gate", "handle_mode_gate"]

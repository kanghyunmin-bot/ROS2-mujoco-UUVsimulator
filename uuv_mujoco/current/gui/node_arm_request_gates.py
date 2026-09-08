"""Gate handling for GUI arm/disarm requests."""

from __future__ import annotations

import time

from .node_command_attempts import should_log_attempt


def handle_arm_target_reached(self, value: bool) -> bool:
    if not self._arm_target_reached(value):
        return False
    self._push_event(f"arm target reached: armed={value}")
    return True


def handle_arm_gate(
    self,
    value: bool,
    deadline: float,
    attempt: int,
    *,
    request_generation: int | None = None,
) -> bool:
    gate_reason = self._arm_mode_gate_reason(arm_value=bool(value))
    if not gate_reason:
        return False
    if time.monotonic() >= deadline:
        self._push_event(f"arm blocked: {gate_reason}")
        return True
    if should_log_attempt(attempt):
        self._push_event(f"arm delayed: {gate_reason}")
    self._retry_arm_request(
        value,
        deadline,
        attempt,
        request_generation=request_generation,
    )
    return True


__all__ = ["handle_arm_gate", "handle_arm_target_reached"]

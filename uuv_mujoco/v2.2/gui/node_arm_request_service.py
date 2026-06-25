"""MAVROS service path for GUI arm/disarm requests."""

from __future__ import annotations

from .node_command_attempts import should_log_attempt
from .runtime_mavros import CommandBool


def arm_service_ready(owner) -> bool:
    if owner._arm_client is None:
        return False
    try:
        return bool(owner._arm_client.service_is_ready())
    except Exception:
        return False


def send_arm_service_request(self, value: bool, deadline: float, attempt: int) -> None:
    if self._arm_request_in_flight:
        self._retry_arm_request(value, deadline, attempt)
        return
    if self._arm_client is None:
        self._push_event("arm service unavailable in current Python env")
        self._retry_arm_request(value, deadline, attempt)
        return
    if not arm_service_ready(self):
        if should_log_attempt(attempt):
            self._push_event("arm service unavailable; waiting")
        self._retry_arm_request(value, deadline, attempt)
        return

    req = CommandBool.Request()
    req.value = bool(value)
    future = self._arm_client.call_async(req)
    self._arm_request_in_flight = True
    future.add_done_callback(
        lambda fut: self._on_arm_response(
            fut,
            "arm" if value else "disarm",
            bool(value),
            deadline,
            attempt,
        )
    )


__all__ = ["arm_service_ready", "send_arm_service_request"]

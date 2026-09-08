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


def send_arm_service_request(
    self,
    value: bool,
    deadline: float,
    attempt: int,
    *,
    request_generation: int | None = None,
) -> bool:
    if self._arm_mode_command_path not in {"auto", "service"}:
        return False
    if self._arm_request_in_flight:
        self._retry_arm_request(
            value,
            deadline,
            attempt,
            request_generation=request_generation,
        )
        return True
    if self._arm_client is None:
        if self._arm_mode_command_path == "auto":
            return False
        self._push_event("arm service unavailable in current Python env")
        self._retry_arm_request(
            value,
            deadline,
            attempt,
            request_generation=request_generation,
        )
        return True
    if not arm_service_ready(self):
        if self._arm_mode_command_path == "auto":
            return False
        if should_log_attempt(attempt):
            self._push_event("arm service unavailable; waiting")
        self._retry_arm_request(
            value,
            deadline,
            attempt,
            request_generation=request_generation,
        )
        return True

    req = CommandBool.Request()
    req.value = bool(value)
    future = self._arm_client.call_async(req)
    self._arm_request_in_flight = True
    self._arm_request_in_flight_generation = request_generation
    future.add_done_callback(
        lambda fut: self._on_arm_response(
            fut,
            "arm" if value else "disarm",
            bool(value),
            deadline,
            attempt,
            request_generation,
        )
    )
    return True


__all__ = ["arm_service_ready", "send_arm_service_request"]

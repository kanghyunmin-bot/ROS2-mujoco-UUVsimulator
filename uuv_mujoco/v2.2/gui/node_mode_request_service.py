"""MAVROS service request path for GUI mode commands."""

from __future__ import annotations

from .node_command_attempts import should_log_attempt
from .runtime_mavros import SetMode


def send_mode_service_request(self, mode: str, deadline: float, attempt: int) -> None:
    if self._mode_request_in_flight:
        self._retry_mode_request(mode, deadline, attempt)
        return
    if self._mode_client is None:
        self._push_event("set_mode service unavailable in current Python env")
        self._retry_mode_request(mode, deadline, attempt)
        return
    try:
        ready = self._mode_client.service_is_ready()
    except Exception:
        ready = False
    if not ready:
        if should_log_attempt(attempt):
            self._push_event("set_mode service unavailable; waiting")
        self._retry_mode_request(mode, deadline, attempt)
        return
    req = SetMode.Request()
    req.base_mode = 0
    req.custom_mode = mode
    future = self._mode_client.call_async(req)
    self._mode_request_in_flight = True
    future.add_done_callback(lambda fut: self._on_mode_response(fut, mode, deadline, attempt))


__all__ = ["send_mode_service_request"]

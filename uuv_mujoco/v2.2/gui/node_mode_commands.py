"""Mode command policy for UuvGuiNode."""

from __future__ import annotations

from .node_mode_request_steps import (
    handle_alt_hold_initial_depth_gate,
    handle_mode_gate,
    publish_mode_override_if_configured,
    send_mode_service_request,
)
from .runtime import *


def set_mode(self, mode: str) -> None:
    mode = str(mode).strip().upper()
    self._latest_mode_target = mode
    self._mode_request_in_flight = False
    deadline = time.monotonic() + self._control_request_timeout_s
    self._send_mode_request(mode, deadline, 1)


def _send_mode_request(self, mode: str, deadline: float, attempt: int) -> None:
    mode = str(mode).strip().upper()
    latest_target = str(getattr(self, "_latest_mode_target", "") or "").strip().upper()
    if latest_target and mode != latest_target:
        return
    if self._mode_target_reached(mode):
        self._push_event(f"mode target reached: {mode}")
        return
    if handle_alt_hold_initial_depth_gate(self, mode, deadline, attempt):
        return
    if handle_mode_gate(self, mode, deadline, attempt):
        return
    if publish_mode_override_if_configured(self, mode, deadline, attempt):
        return
    send_mode_service_request(self, mode, deadline, attempt)


def _on_mode_response(self, future, mode: str, deadline: float, attempt: int) -> None:
    self._mode_request_in_flight = False
    latest_target = str(getattr(self, "_latest_mode_target", "") or "").strip().upper()
    if latest_target and str(mode).strip().upper() != latest_target:
        return
    try:
        resp = future.result()
    except Exception as exc:
        self._push_event(f"set_mode {mode} failed: {exc}")
        self._retry_mode_request(mode, deadline, attempt)
        return
    self._push_event(f"set_mode {mode}: mode_sent={resp.mode_sent}, attempt={attempt}")
    if self._mode_target_reached(mode):
        self._push_event(f"mode target reached: {mode}")
        return
    self._retry_mode_request(mode, deadline, attempt)


__all__ = ["_on_mode_response", "_send_mode_request", "set_mode"]

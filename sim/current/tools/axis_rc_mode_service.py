"""SetMode service helper for axis RC checks."""

from __future__ import annotations

import time
from typing import Any

from mavros_msgs.srv import SetMode

from axis_rc_service_spin import publish_neutral_and_spin, wait_for_future


def call_set_mode(node: Any, mode: str, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    mode_sent = False
    while time.monotonic() < deadline:
        if _state_reports_mode(node, mode):
            return
        future = node.mode_client.call_async(_set_mode_request(mode))
        wait_for_future(node, future, min(deadline, time.monotonic() + 2.0))
        mode_sent = _mode_request_sent(future)
        if not mode_sent:
            publish_neutral_and_spin(node, timeout_sec=0.05)
            if _state_reports_mode(node, mode):
                return
            continue
        publish_neutral_and_spin(node, timeout_sec=0.05)
        if _state_reports_mode(node, mode):
            return
    reason = "service rejected request" if not mode_sent else "state did not report requested mode"
    raise RuntimeError(f"set_mode({mode}) failed: {reason}")


def _set_mode_request(mode: str) -> SetMode.Request:
    req = SetMode.Request()
    req.base_mode = 0
    req.custom_mode = str(mode)
    return req


def _mode_request_sent(future: Any) -> bool:
    return future.done() and future.result() is not None and bool(future.result().mode_sent)


def _state_reports_mode(node: Any, mode: str) -> bool:
    return node.state is not None and str(node.state.mode) == mode


__all__ = ["call_set_mode"]

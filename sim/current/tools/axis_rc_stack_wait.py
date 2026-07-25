"""Stack readiness wait helpers for axis RC checks."""

from __future__ import annotations

import time
from typing import Any

from axis_rc_service_spin import publish_neutral_and_spin


def wait_for_stack(node: Any, timeout: float, *, require_manual_input: bool = False) -> None:
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        if _stack_ready(node, require_manual_input=require_manual_input):
            return
        publish_neutral_and_spin(node, timeout_sec=0.05)
    if require_manual_input:
        raise RuntimeError("MAVROS-like surface did not become ready: RC override command link is not ready")
    raise RuntimeError("MAVROS-like surface did not become ready")


def _stack_ready(node: Any, *, require_manual_input: bool) -> bool:
    return (
        _services_ready(node)
        and _state_ready(node)
        and _manual_input_ready(node, require_manual_input=require_manual_input)
    )


def _services_ready(node: Any) -> bool:
    return node.arm_client.wait_for_service(timeout_sec=0.1) and node.mode_client.wait_for_service(
        timeout_sec=0.1
    )


def _state_ready(node: Any) -> bool:
    return node.state is not None and bool(node.state.connected) and bool(str(node.state.mode).strip())


def _manual_input_ready(node: Any, *, require_manual_input: bool) -> bool:
    if not require_manual_input:
        return True
    return node.state is not None and bool(getattr(node.state, "manual_input", False))


__all__ = ["wait_for_stack"]

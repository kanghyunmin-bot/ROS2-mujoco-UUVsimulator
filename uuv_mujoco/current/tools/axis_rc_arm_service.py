"""Arming service helper for axis RC checks."""

from __future__ import annotations

import time
from typing import Any

from axis_rc_arm_state import (
    arm_failure_reason,
    arm_request_accepted,
    build_arm_request,
    state_reports_arm,
)
from axis_rc_service_spin import publish_neutral_and_spin, spin_neutral_then_sleep, wait_for_future


def call_arm(node: Any, armed: bool, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    accepted_any = False
    while time.monotonic() < deadline:
        if state_reports_arm(node, armed):
            return
        if accepted_any:
            spin_neutral_then_sleep(node, spin_timeout_sec=0.1, sleep_s=0.05)
            continue
        future = node.arm_client.call_async(build_arm_request(armed))
        wait_for_future(node, future, min(deadline, time.monotonic() + 2.0))
        accepted = arm_request_accepted(future)
        accepted_any = accepted_any or accepted
        if not accepted_any:
            spin_neutral_then_sleep(node, spin_timeout_sec=0.1, sleep_s=0.1)
            continue
        publish_neutral_and_spin(node, timeout_sec=0.05)
        if state_reports_arm(node, armed):
            return
    reason = arm_failure_reason(accepted_any)
    raise RuntimeError(f"arming({armed}) failed: {reason}")


__all__ = ["call_arm"]

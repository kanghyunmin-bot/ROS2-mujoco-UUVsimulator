"""Arming request and state predicates for axis RC checks."""

from __future__ import annotations

from typing import Any

from mavros_msgs.srv import CommandBool


def build_arm_request(armed: bool) -> CommandBool.Request:
    req = CommandBool.Request()
    req.value = bool(armed)
    return req


def arm_request_accepted(future: Any) -> bool:
    return future.done() and future.result() is not None and bool(future.result().success)


def state_reports_arm(node: Any, armed: bool) -> bool:
    return node.state is not None and bool(node.state.armed) == bool(armed)


def arm_failure_reason(accepted_any: bool) -> str:
    if not accepted_any:
        return "service rejected request"
    return "state did not report requested arm state"


__all__ = [
    "arm_failure_reason",
    "arm_request_accepted",
    "build_arm_request",
    "state_reports_arm",
]

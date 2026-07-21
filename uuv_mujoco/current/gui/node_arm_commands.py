"""Arm/disarm command policy for UuvGuiNode."""

from __future__ import annotations

from .config import BACKEND_SIM_BRIDGE
from .node_arm_request_steps import (
    arm_deadline,
    handle_arm_gate,
    handle_arm_target_reached,
    publish_arm_override_if_configured,
    send_arm_service_request,
)
from .runtime import *


ARM_STATE_CONFIRM_GRACE_S = 1.25


def _confirm_arm_target_or_retry(
    self,
    target_value: bool,
    deadline: float,
    attempt: int,
) -> None:
    latest_target = getattr(self, "_latest_arm_target", None)
    if latest_target is not None and bool(target_value) != bool(latest_target):
        return
    if handle_arm_target_reached(self, target_value):
        return
    if time.monotonic() >= deadline:
        self._push_event(f"arm target timeout: armed={target_value}")
        return
    self._send_arm_request(target_value, deadline, attempt + 1)


def _send_arm_request(self, value: bool, deadline: Optional[float] = None, attempt: int = 1) -> None:
    deadline = arm_deadline(self, deadline)
    latest_target = getattr(self, "_latest_arm_target", None)
    if latest_target is not None and bool(value) != bool(latest_target):
        return
    if handle_arm_target_reached(self, value):
        return
    if handle_arm_gate(self, value, deadline, attempt):
        return
    if _prefer_command_override_topic(self) and publish_arm_override_if_configured(self, value, deadline, attempt):
        return
    if send_arm_service_request(self, value, deadline, attempt):
        return
    if publish_arm_override_if_configured(self, value, deadline, attempt):
        return


def arm(self, value: bool) -> None:
    self._latest_arm_target = bool(value)
    self._arm_request_in_flight = False
    deadline = time.monotonic() + self._control_request_timeout_s
    self._send_arm_request(value, deadline, 1)


def _prefer_command_override_topic(self) -> bool:
    return self._arm_mode_command_path == "auto" and self._effective_backend() == BACKEND_SIM_BRIDGE


def _on_arm_response(
    self,
    future,
    action: str,
    target_value: bool,
    deadline: float,
    attempt: int,
) -> None:
    self._arm_request_in_flight = False
    latest_target = getattr(self, "_latest_arm_target", None)
    if latest_target is not None and bool(target_value) != bool(latest_target):
        return
    try:
        resp = future.result()
    except Exception as exc:
        self._push_event(f"{action} failed: {exc}")
        self._retry_arm_request(target_value, deadline, attempt)
        return
    self._push_event(f"{action}: success={resp.success}, result={resp.result}, attempt={attempt}")
    if self._arm_target_reached(target_value):
        self._push_event(f"arm target reached: armed={target_value}")
        return
    if bool(resp.success):
        # COMMAND_ACK normally arrives in about 100 ms, while /mavros/state is
        # heartbeat-driven and can take up to roughly one second to show the
        # new armed bit.  The old 50 ms retry loop sent 10-20 duplicate arm
        # commands during that harmless feedback interval.  Wait one heartbeat
        # window before retrying an already accepted command.
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0.0:
            self._push_event(f"arm target timeout: armed={target_value}")
            return
        self._schedule_once(
            min(ARM_STATE_CONFIRM_GRACE_S, remaining_s),
            lambda: _confirm_arm_target_or_retry(
                self,
                target_value,
                deadline,
                attempt,
            ),
        )
        return
    self._retry_arm_request(target_value, deadline, attempt)


__all__ = [
    "ARM_STATE_CONFIRM_GRACE_S",
    "_confirm_arm_target_or_retry",
    "_on_arm_response",
    "_prefer_command_override_topic",
    "_send_arm_request",
    "arm",
]

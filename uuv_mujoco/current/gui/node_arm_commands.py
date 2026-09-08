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
from .node_arm_rc_sequence import (
    arm_command_request_current,
    arm_rc_sequence_failed,
    cancel_arm_rc_sequence,
    current_arm_command_generation,
    ensure_arm_rc_precondition,
    finish_arm_command_generation,
    on_arm_state_observed,
    prepare_arm_rc_sequence_for_target,
    publish_arm_ack_neutral,
)
from .runtime import *


ARM_STATE_CONFIRM_GRACE_S = 1.25


def _confirm_arm_target_or_retry(
    self,
    target_value: bool,
    deadline: float,
    attempt: int,
    request_generation: int | None = None,
) -> None:
    if request_generation is None:
        request_generation = current_arm_command_generation(self)
    with self._arm_rc_sequence_lock:
        if not arm_command_request_current(
            self,
            target_value,
            request_generation,
        ):
            return
        _confirm_current_arm_target_or_retry(
            self,
            target_value,
            deadline,
            attempt,
            request_generation,
        )


def _confirm_current_arm_target_or_retry(
    self,
    target_value: bool,
    deadline: float,
    attempt: int,
    request_generation: int,
) -> None:
    if handle_arm_target_reached(self, target_value):
        finish_arm_command_generation(self, request_generation)
        cancel_arm_rc_sequence(
            self,
            f"arm target reached: armed={bool(target_value)}",
            force_neutral=True,
        )
        return
    if time.monotonic() >= deadline:
        finish_arm_command_generation(self, request_generation)
        cancel_arm_rc_sequence(
            self,
            f"arm target timeout: armed={bool(target_value)}",
            force_neutral=True,
        )
        self._push_event(f"arm target timeout: armed={target_value}")
        return
    # An accepted service ACK already restored RC3=1500.  Invalidate that
    # settle generation so any actual resend performs a fresh low-RC3 window.
    cancel_arm_rc_sequence(
        self,
        "arming state not yet confirmed; retrying",
        force_neutral=True,
    )
    self._send_arm_request(
        target_value,
        deadline,
        attempt + 1,
        request_generation=request_generation,
    )


def _send_arm_request(
    self,
    value: bool,
    deadline: Optional[float] = None,
    attempt: int = 1,
    *,
    request_generation: int | None = None,
) -> None:
    deadline = arm_deadline(self, deadline)
    if request_generation is None:
        request_generation = current_arm_command_generation(self)
    with self._arm_rc_sequence_lock:
        if not arm_command_request_current(self, value, request_generation):
            return
        _send_current_arm_request(
            self,
            value,
            float(deadline),
            int(attempt),
            int(request_generation),
        )


def _send_current_arm_request(
    self,
    value: bool,
    deadline: float,
    attempt: int,
    request_generation: int,
) -> None:
    if handle_arm_target_reached(self, value):
        finish_arm_command_generation(self, request_generation)
        cancel_arm_rc_sequence(
            self,
            f"arm target reached: armed={bool(value)}",
            force_neutral=True,
        )
        return
    if handle_arm_gate(
        self,
        value,
        deadline,
        attempt,
        request_generation=request_generation,
    ):
        if time.monotonic() >= deadline:
            finish_arm_command_generation(self, request_generation)
            cancel_arm_rc_sequence(
                self,
                f"arm gate timeout: armed={bool(value)}",
                force_neutral=True,
            )
        return
    if bool(value) and ensure_arm_rc_precondition(
        self,
        deadline=float(deadline),
        attempt=int(attempt),
        command_generation=int(request_generation),
    ):
        return
    if _prefer_command_override_topic(self) and publish_arm_override_if_configured(
        self,
        value,
        deadline,
        attempt,
        request_generation=request_generation,
    ):
        return
    if send_arm_service_request(
        self,
        value,
        deadline,
        attempt,
        request_generation=request_generation,
    ):
        return
    if publish_arm_override_if_configured(
        self,
        value,
        deadline,
        attempt,
        request_generation=request_generation,
    ):
        return
    finish_arm_command_generation(self, request_generation)
    arm_rc_sequence_failed(self, "no arming service or topic command path")
    self._push_event("arm failed: no arming service or topic command path")


def arm(self, value: bool) -> None:
    requested = bool(value)
    current_arm_command_generation(self)
    with self._arm_rc_sequence_lock:
        if (
            getattr(self, "_latest_arm_target", None) is requested
            and bool(self._arm_command_pending)
        ):
            self._push_event(f"arm request already pending: armed={requested}")
            return
        self._latest_arm_target = requested
        self._arm_request_in_flight = False
        request_generation = prepare_arm_rc_sequence_for_target(self, requested)
        self._arm_command_pending = True
    deadline = time.monotonic() + self._control_request_timeout_s
    self._send_arm_request(
        requested,
        deadline,
        1,
        request_generation=request_generation,
    )


def _on_arm_state_observed(self, armed: bool) -> None:
    """Apply the immediate post-confirmation RC3 neutral transition."""

    on_arm_state_observed(self, bool(armed))


def _prefer_command_override_topic(self) -> bool:
    return self._arm_mode_command_path == "auto" and self._effective_backend() == BACKEND_SIM_BRIDGE


def _on_arm_response(
    self,
    future,
    action: str,
    target_value: bool,
    deadline: float,
    attempt: int,
    request_generation: int | None = None,
) -> None:
    if request_generation is None:
        request_generation = current_arm_command_generation(self)
    with self._arm_rc_sequence_lock:
        if not arm_command_request_current(
            self,
            target_value,
            request_generation,
        ):
            return
        in_flight_generation = getattr(
            self,
            "_arm_request_in_flight_generation",
            request_generation,
        )
        if (
            in_flight_generation is not None
            and int(in_flight_generation) != int(request_generation)
        ):
            return
        _handle_current_arm_response(
            self,
            future,
            action,
            target_value,
            deadline,
            attempt,
            int(request_generation),
        )


def _handle_current_arm_response(
    self,
    future,
    action: str,
    target_value: bool,
    deadline: float,
    attempt: int,
    request_generation: int,
) -> None:
    self._arm_request_in_flight = False
    self._arm_request_in_flight_generation = None
    try:
        resp = future.result()
    except Exception as exc:
        self._push_event(f"{action} failed: {exc}")
        arm_rc_sequence_failed(self, f"{action} transport failure")
        self._retry_arm_request(
            target_value,
            deadline,
            attempt,
            request_generation=request_generation,
        )
        return
    self._push_event(f"{action}: success={resp.success}, result={resp.result}, attempt={attempt}")
    if self._arm_target_reached(target_value):
        finish_arm_command_generation(self, request_generation)
        cancel_arm_rc_sequence(
            self,
            f"arm target reached: armed={bool(target_value)}",
            force_neutral=True,
        )
        self._push_event(f"arm target reached: armed={target_value}")
        return
    if bool(resp.success):
        if bool(target_value):
            publish_arm_ack_neutral(self, f"{action} service accepted")
        # COMMAND_ACK normally arrives in about 100 ms, while /mavros/state is
        # heartbeat-driven and can take up to roughly one second to show the
        # new armed bit.  The old 50 ms retry loop sent 10-20 duplicate arm
        # commands during that harmless feedback interval.  Wait one heartbeat
        # window before retrying an already accepted command.
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0.0:
            finish_arm_command_generation(self, request_generation)
            cancel_arm_rc_sequence(
                self,
                f"arm target timeout: armed={bool(target_value)}",
                force_neutral=True,
            )
            self._push_event(f"arm target timeout: armed={target_value}")
            return
        self._schedule_once(
            min(ARM_STATE_CONFIRM_GRACE_S, remaining_s),
            lambda: _confirm_arm_target_or_retry(
                self,
                target_value,
                deadline,
                attempt,
                request_generation,
            ),
        )
        return
    arm_rc_sequence_failed(self, f"{action} request rejected")
    self._retry_arm_request(
        target_value,
        deadline,
        attempt,
        request_generation=request_generation,
    )


__all__ = [
    "ARM_STATE_CONFIRM_GRACE_S",
    "_confirm_arm_target_or_retry",
    "_on_arm_state_observed",
    "_on_arm_response",
    "_prefer_command_override_topic",
    "_send_arm_request",
    "arm",
]

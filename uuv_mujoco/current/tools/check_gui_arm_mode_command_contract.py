#!/usr/bin/env python3
"""Regression checks for GUI arm/mode command request routing."""

from __future__ import annotations

from pathlib import Path
import sys
import time
from types import ModuleType
from typing import Optional


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

runtime_stub = ModuleType("gui.runtime")
runtime_stub.time = time
runtime_stub.Optional = Optional
sys.modules.setdefault("gui.runtime", runtime_stub)

from gui.node_arm_commands import ARM_STATE_CONFIRM_GRACE_S, _on_arm_response, arm  # noqa: E402
from gui.node_arm_request_service import send_arm_service_request  # noqa: E402
from gui.node_arm_request_topic import publish_arm_override_if_configured  # noqa: E402
from gui.node_command_retries import _retry_arm_request, _retry_mode_request  # noqa: E402
from gui.node_mode_commands import set_mode  # noqa: E402
from gui.node_mode_request_service import send_mode_service_request  # noqa: E402
from gui.node_mode_request_topic import publish_mode_override_if_configured  # noqa: E402


class FakeFuture:
    def __init__(self, response=None) -> None:
        self.callbacks = []
        self.response = response

    def add_done_callback(self, callback) -> None:
        self.callbacks.append(callback)

    def result(self):
        return self.response


class FakeClient:
    def __init__(self, ready: bool = True) -> None:
        self.ready = ready
        self.requests = []
        self.futures = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = FakeFuture()
        self.futures.append(future)
        return future


class FakeOwner:
    def __init__(self) -> None:
        self._arm_mode_command_path = "topic"
        self._arm_client = FakeClient()
        self._mode_client = FakeClient()
        self._arm_request_in_flight = False
        self._mode_request_in_flight = False
        self._control_request_timeout_s = 20.0
        self._control_request_retry_s = 0.05
        self.published_payloads = []
        self.scheduled = []
        self.events = []
        self.arm_requests = []
        self.mode_requests = []
        self.initial_depth_releases = []
        self.arm_target_reached = False

    def _publish_command_override(self, payload: dict[str, object]) -> bool:
        self.published_payloads.append(dict(payload))
        return True

    def _schedule_once(self, delay_s: float, callback) -> None:
        self.scheduled.append((delay_s, callback))

    def _push_event(self, text: str) -> None:
        self.events.append(text)

    def _arm_mode_gate_reason(self, *, arm_value=None, mode: str = "") -> str:
        del arm_value, mode
        return ""

    def _arm_target_reached(self, value: bool) -> bool:
        del value
        return self.arm_target_reached

    def _mode_target_reached(self, mode: str) -> bool:
        del mode
        return False

    def _request_initial_depth_release_when_armed(self, reason: str) -> None:
        self.initial_depth_releases.append(reason)

    def _send_arm_request(
        self,
        value: bool,
        deadline=None,
        attempt: int = 1,
        *,
        request_generation: int | None = None,
    ) -> None:
        self.arm_requests.append(
            (bool(value), deadline, int(attempt), request_generation)
        )

    def _send_mode_request(self, mode: str, deadline: float, attempt: int) -> None:
        self.mode_requests.append((str(mode), float(deadline), int(attempt)))

    def _retry_arm_request(
        self,
        value: bool,
        deadline: float,
        attempt: int,
        *,
        request_generation: int | None = None,
    ) -> None:
        self.scheduled.append(
            ("arm", bool(value), float(deadline), int(attempt), request_generation)
        )

    @staticmethod
    def publish_rc_arm_low() -> bool:
        return True

    @staticmethod
    def publish_rc_arm_neutral() -> bool:
        return True

    def _retry_mode_request(self, mode: str, deadline: float, attempt: int) -> None:
        self.scheduled.append(("mode", str(mode), float(deadline), int(attempt)))

    def _on_arm_response(self, *_args, **_kwargs) -> None:
        raise AssertionError("callback should not fire during request-build smoke")

    def _on_mode_response(self, *_args, **_kwargs) -> None:
        raise AssertionError("callback should not fire during request-build smoke")


def _assert(condition: bool, label: str) -> None:
    if not condition:
        raise AssertionError(label)


def check_high_level_alt_hold_release_policy() -> None:
    owner = FakeOwner()
    arm(owner, True)
    _assert(not owner.initial_depth_releases, "arm(true) must not release initial-depth hold")
    _assert(owner.arm_requests and owner.arm_requests[-1][0] is True, "arm(true) must queue an arm request")

    owner = FakeOwner()
    set_mode(owner, "ALT_HOLD")
    _assert(not owner.initial_depth_releases, "ALT_HOLD mode request must not release initial-depth hold")
    _assert(owner.mode_requests and owner.mode_requests[-1][0] == "ALT_HOLD", "ALT_HOLD must queue a mode request")

    owner = FakeOwner()
    set_mode(owner, "MANUAL")
    _assert(not owner.initial_depth_releases, "MANUAL mode request must not release initial-depth hold")


def check_topic_command_path() -> None:
    owner = FakeOwner()
    sent = publish_arm_override_if_configured(owner, True, 100.0, 1)
    _assert(sent, "topic arm path must report sent")
    _assert(owner.published_payloads == [{"arm": True}], "topic arm payload mismatch")

    owner = FakeOwner()
    sent = publish_mode_override_if_configured(owner, "ALT_HOLD", 100.0, 1)
    _assert(sent, "topic mode path must report sent")
    _assert(owner.published_payloads == [{"mode": "ALT_HOLD"}], "topic mode payload mismatch")


def check_service_command_path() -> None:
    owner = FakeOwner()
    owner._arm_mode_command_path = "service"
    send_arm_service_request(owner, True, 100.0, 1)
    _assert(owner._arm_request_in_flight, "arm service request must mark in-flight")
    _assert(len(owner._arm_client.requests) == 1, "arm service request count mismatch")
    _assert(owner._arm_client.requests[0].value is True, "arm service request value mismatch")

    owner = FakeOwner()
    owner._arm_mode_command_path = "service"
    send_mode_service_request(owner, "ALT_HOLD", 100.0, 1)
    _assert(owner._mode_request_in_flight, "mode service request must mark in-flight")
    _assert(len(owner._mode_client.requests) == 1, "mode service request count mismatch")
    req = owner._mode_client.requests[0]
    _assert(req.base_mode == 0, "mode service base_mode must be 0")
    _assert(req.custom_mode == "ALT_HOLD", "mode service custom_mode mismatch")


def check_stale_retries_are_dropped() -> None:
    owner = FakeOwner()
    owner._latest_mode_target = "ALT_HOLD"
    _retry_mode_request(owner, "MANUAL", time.monotonic() + 5.0, 1)
    _assert(not owner.scheduled, "stale mode retry must not be scheduled")

    _retry_mode_request(owner, "ALT_HOLD", time.monotonic() + 5.0, 1)
    _assert(len(owner.scheduled) == 1, "current mode retry must be scheduled")
    owner._latest_mode_target = "SURFACE"
    owner.scheduled[0][1]()
    _assert(not owner.mode_requests, "scheduled stale mode callback must not send")

    owner = FakeOwner()
    owner._latest_arm_target = True
    _retry_arm_request(owner, False, time.monotonic() + 5.0, 1)
    _assert(not owner.scheduled, "stale arm retry must not be scheduled")

    _retry_arm_request(owner, True, time.monotonic() + 5.0, 1)
    _assert(len(owner.scheduled) == 1, "current arm retry must be scheduled")
    owner._latest_arm_target = False
    owner.scheduled[0][1]()
    _assert(not owner.arm_requests, "scheduled stale arm callback must not send")


def check_accepted_arm_waits_for_state_heartbeat() -> None:
    owner = FakeOwner()
    owner._latest_arm_target = True
    response = type("Response", (), {"success": True, "result": 0})()
    deadline = time.monotonic() + 5.0

    _on_arm_response(owner, FakeFuture(response), "arm", True, deadline, 1)

    _assert(len(owner.scheduled) == 1, "accepted arm must schedule one confirmation check")
    delay_s, callback = owner.scheduled[0]
    _assert(
        float(delay_s) == ARM_STATE_CONFIRM_GRACE_S,
        "accepted arm confirmation must wait one state-heartbeat window",
    )
    _assert(not owner.arm_requests, "accepted arm must not immediately resend")

    owner.arm_target_reached = True
    callback()
    _assert(not owner.arm_requests, "confirmed arm must not resend after heartbeat")


def check_web_arm_requests_fast_status_confirmation() -> None:
    app_js = (ROOT / "gui" / "web_static" / "app.js").read_text(encoding="utf-8")
    _assert("function pollArmTransition()" in app_js, "web arm transition poll helper missing")
    _assert("window.setTimeout(pollStatus, delayMs)" in app_js, "web arm fast status poll missing")
    _assert('requestArm(true)' in app_js, "Arm button must use requestArm")
    _assert('requestArm(false)' in app_js, "Disarm button must use requestArm")


def main() -> int:
    check_high_level_alt_hold_release_policy()
    check_topic_command_path()
    check_service_command_path()
    check_stale_retries_are_dropped()
    check_accepted_arm_waits_for_state_heartbeat()
    check_web_arm_requests_fast_status_confirmation()
    print("gui_arm_mode_command_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

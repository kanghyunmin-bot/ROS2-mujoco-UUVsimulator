#!/usr/bin/env python3
"""Deterministic regression harness for the ArduSub 4.1.2 arm RC3 sequence."""

from __future__ import annotations

import heapq
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import unittest
from unittest.mock import patch


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from gui.config import BACKEND_MAVROS, RC_LAYOUTS  # noqa: E402
from gui.node_arm_commands import (  # noqa: E402
    _on_arm_response,
    _on_arm_state_observed,
    _send_arm_request,
    arm,
)
from gui.node_arm_rc_sequence import (  # noqa: E402
    ARM_LOW_RC3_PWM,
    ARM_NEUTRAL_RC3_PWM,
    initialize_arm_rc_sequence_state,
)
from gui.node_command_retries import _retry_arm_request  # noqa: E402
from gui.node_manual_control_publishers import publish_manual_control  # noqa: E402
from gui.node_rc_override_publishers import (  # noqa: E402
    publish_rc_arm_low,
    publish_rc_arm_neutral,
    publish_rc_channels,
    publish_rc_override,
    publish_rc_release,
)
from gui.node_vehicle_callbacks import _on_state  # noqa: E402


class FakePublisher:
    def __init__(self, owner) -> None:
        self.owner = owner

    def publish(self, message) -> None:
        channels = tuple(int(value) for value in message.channels)
        self.owner.timeline.append(("rc", channels))


class FakeFuture:
    def __init__(self) -> None:
        self.callback = None
        self.response = SimpleNamespace(success=True, result=0)
        self.error: Exception | None = None

    def add_done_callback(self, callback) -> None:
        self.callback = callback

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response

    def finish(self, *, success: bool = True, error: Exception | None = None) -> None:
        self.response = SimpleNamespace(success=bool(success), result=0)
        self.error = error
        if self.callback is None:
            raise AssertionError("arm future has no completion callback")
        self.callback(self)


class FakeArmClient:
    def __init__(self, owner) -> None:
        self.owner = owner
        self.futures: list[FakeFuture] = []

    @staticmethod
    def service_is_ready() -> bool:
        return True

    def call_async(self, request) -> FakeFuture:
        self.owner.timeline.append(("arm_request", bool(request.value)))
        future = FakeFuture()
        self.futures.append(future)
        return future


class FakeManualPublisher:
    def __init__(self, owner) -> None:
        self.owner = owner

    def publish(self, message) -> None:
        self.owner.timeline.append(
            ("manual", (message.x, message.y, message.z, message.r))
        )


class HarnessOwner:
    arm = arm
    _send_arm_request = _send_arm_request
    _on_arm_response = _on_arm_response
    _on_arm_state_observed = _on_arm_state_observed
    _retry_arm_request = _retry_arm_request
    publish_rc_arm_low = publish_rc_arm_low
    publish_rc_arm_neutral = publish_rc_arm_neutral
    publish_rc_override = publish_rc_override
    publish_rc_channels = publish_rc_channels
    publish_rc_release = publish_rc_release
    publish_manual_control = publish_manual_control

    def __init__(self) -> None:
        self.now = 10.0
        self.timeline: list[tuple[str, object]] = []
        self.events: list[str] = []
        self._timers: list[tuple[float, int, object]] = []
        self._timer_order = 0
        self._lock = threading.RLock()
        self._snapshot = SimpleNamespace(
            connected=True,
            armed=False,
            guided=False,
            manual_input=False,
            mode="STABILIZE",
            system_status=3,
        )
        self._last_wall = {"state": self.now}
        self._last_mode_seen = "STABILIZE"
        self._last_armed_seen = False
        self._vehicle_connected_since_wall = 1.0
        self._latest_arm_target = None
        self._arm_request_in_flight = False
        self._control_request_timeout_s = 1.0
        self._control_request_retry_s = 0.05
        self._arm_mode_command_path = "service"
        self._arm_client = FakeArmClient(self)
        self._rc_override_publisher_lock = threading.RLock()
        self._rc_override_pub = FakePublisher(self)
        self._manual_control_pub = FakeManualPublisher(self)
        self._rc_override_burst_count = 1
        self._rc_override_publish_generation = 0
        self._arm_rc3_low_repeat_count = 3
        self._arm_rc3_low_settle_s = 0.12
        initialize_arm_rc_sequence_state(self)

    @staticmethod
    def _effective_backend() -> str:
        return BACKEND_MAVROS

    @staticmethod
    def _active_layout():
        return RC_LAYOUTS[BACKEND_MAVROS]

    @staticmethod
    def _publish_command_override(_payload) -> bool:
        return False

    @staticmethod
    def _arm_mode_gate_reason(*, arm_value=None, mode="") -> str:
        del arm_value
        del mode
        return ""

    def _arm_target_reached(self, value: bool) -> bool:
        return bool(self._snapshot.connected) and bool(self._snapshot.armed) == bool(value)

    def _schedule_once(self, delay_s: float, callback) -> None:
        self._timer_order += 1
        heapq.heappush(
            self._timers,
            (self.now + max(0.0, float(delay_s)), self._timer_order, callback),
        )

    def advance(self, delta_s: float) -> None:
        target = self.now + float(delta_s)
        while self._timers and self._timers[0][0] <= target:
            due, _order, callback = heapq.heappop(self._timers)
            self.now = due
            callback()
        self.now = target

    def _push_event(self, text: str) -> None:
        self.events.append(str(text))
        self.timeline.append(("event", str(text)))

    def _touch(self, key: str) -> None:
        self._last_wall[str(key)] = self.now

    @staticmethod
    def _try_release_initial_depth_hold() -> None:
        return None


def rc3_of(entry: tuple[str, object]) -> int:
    return int(entry[1][2])


def primary_channels(entry: tuple[str, object]) -> tuple[int, ...]:
    return tuple(int(value) for value in entry[1][:8])


PRIMARY_NEUTRAL = (1500, 1500, 1500, 1500, 1500, 1500, 65535, 65535)
PRIMARY_ARM_LOW = (1500, 1500, 1100, 1500, 1500, 1500, 65535, 65535)


class GuiArmRc3SequenceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.owner = HarnessOwner()
        patcher = patch("time.monotonic", side_effect=lambda: self.owner.now)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_success_orders_low_rc3_before_request_and_neutral_after_state(self) -> None:
        self.owner.arm(True)
        self.assertFalse(any(kind == "arm_request" for kind, _ in self.owner.timeline))

        self.owner.advance(0.13)
        request_index = next(
            index
            for index, entry in enumerate(self.owner.timeline)
            if entry == ("arm_request", True)
        )
        low_entries = [
            entry
            for entry in self.owner.timeline[:request_index]
            if entry[0] == "rc" and rc3_of(entry) == ARM_LOW_RC3_PWM
        ]
        self.assertGreaterEqual(len(low_entries), 3)
        for entry in low_entries:
            self.assertEqual(primary_channels(entry), PRIMARY_ARM_LOW)

        self.owner._arm_client.futures[-1].finish(success=True)
        ack_rc = next(entry for entry in reversed(self.owner.timeline) if entry[0] == "rc")
        self.assertEqual(primary_channels(ack_rc), PRIMARY_NEUTRAL)

        state = SimpleNamespace(
            connected=True,
            armed=True,
            guided=False,
            manual_input=False,
            mode="STABILIZE",
            system_status=4,
        )
        _on_state(self.owner, state)

        final_rc = next(entry for entry in reversed(self.owner.timeline) if entry[0] == "rc")
        self.assertEqual(rc3_of(final_rc), ARM_NEUTRAL_RC3_PWM)
        self.assertEqual(primary_channels(final_rc), PRIMARY_NEUTRAL)
        self.assertGreater(self.owner.timeline.index(final_rc), request_index)

    def test_armed_state_cannot_be_followed_by_stale_low_pulse(self) -> None:
        self.owner.arm(True)
        _due, _order, pulse = heapq.heappop(self.owner._timers)
        low_entered = threading.Event()
        allow_low = threading.Event()
        state_done = threading.Event()

        from gui import node_arm_rc_sequence as arm_sequence

        original_publish_low = arm_sequence._publish_arm_low

        def blocked_publish_low(owner) -> bool:
            low_entered.set()
            self.assertTrue(allow_low.wait(timeout=1.0))
            return original_publish_low(owner)

        state = SimpleNamespace(
            connected=True,
            armed=True,
            guided=False,
            manual_input=False,
            mode="STABILIZE",
            system_status=4,
        )

        with patch.object(arm_sequence, "_publish_arm_low", blocked_publish_low):
            pulse_thread = threading.Thread(target=pulse)
            pulse_thread.start()
            self.assertTrue(low_entered.wait(timeout=1.0))
            state_thread = threading.Thread(
                target=lambda: (_on_state(self.owner, state), state_done.set())
            )
            state_thread.start()
            self.assertFalse(
                state_done.wait(timeout=0.05),
                "armed observer must serialize behind an in-progress low publish",
            )
            allow_low.set()
            pulse_thread.join(timeout=1.0)
            state_thread.join(timeout=1.0)

        self.assertFalse(pulse_thread.is_alive())
        self.assertFalse(state_thread.is_alive())
        final_rc = next(entry for entry in reversed(self.owner.timeline) if entry[0] == "rc")
        self.assertEqual(primary_channels(final_rc), PRIMARY_NEUTRAL)

    def test_true_false_true_aba_drops_old_service_future(self) -> None:
        self.owner.arm(True)
        self.owner.advance(0.13)
        old_future = self.owner._arm_client.futures[-1]

        self.owner.arm(False)
        self.owner.arm(True)
        self.owner.advance(0.13)
        self.assertEqual(len(self.owner._arm_client.futures), 2)
        current_generation = self.owner._arm_command_generation
        current_sequence = self.owner._arm_rc_sequence_generation
        rc_count = sum(entry[0] == "rc" for entry in self.owner.timeline)

        old_future.finish(success=False)

        self.assertEqual(self.owner._arm_command_generation, current_generation)
        self.assertEqual(self.owner._arm_rc_sequence_generation, current_sequence)
        self.assertTrue(self.owner._arm_request_in_flight)
        self.assertEqual(
            self.owner._arm_request_in_flight_generation,
            current_generation,
        )
        self.assertEqual(
            sum(entry[0] == "rc" for entry in self.owner.timeline),
            rc_count,
        )

    def test_repeated_arm_true_is_idempotent_while_pending(self) -> None:
        self.owner.arm(True)
        generation = self.owner._arm_command_generation
        timer_count = len(self.owner._timers)
        rc_count = sum(entry[0] == "rc" for entry in self.owner.timeline)

        self.owner.arm(True)

        self.assertEqual(self.owner._arm_command_generation, generation)
        self.assertEqual(len(self.owner._timers), timer_count)
        self.assertEqual(
            sum(entry[0] == "rc" for entry in self.owner.timeline),
            rc_count,
        )
        self.owner.advance(0.13)
        self.assertEqual(len(self.owner._arm_client.futures), 1)

    def test_service_failure_restores_neutral_before_retry(self) -> None:
        self.owner.arm(True)
        self.owner.advance(0.13)
        self.owner._arm_client.futures[-1].finish(success=False)

        final_rc = next(entry for entry in reversed(self.owner.timeline) if entry[0] == "rc")
        self.assertEqual(primary_channels(final_rc), PRIMARY_NEUTRAL)

    def test_timeout_restores_neutral(self) -> None:
        self.owner.arm(True)
        self.owner.advance(1.01)

        final_rc = next(entry for entry in reversed(self.owner.timeline) if entry[0] == "rc")
        self.assertEqual(primary_channels(final_rc), PRIMARY_NEUTRAL)
        self.assertTrue(any("timeout" in event for event in self.owner.events))

    def test_disarm_cancels_pending_low_sequence_before_disarm_request(self) -> None:
        self.owner.arm(True)
        # Exercise the real disarm transport path while a pre-arm generation
        # is pending.  Without this state change, an already-disarmed vehicle
        # correctly treats arm(False) as target-reached and sends no request.
        self.owner._snapshot.armed = True
        self.owner.arm(False)

        neutral_index = max(
            index
            for index, entry in enumerate(self.owner.timeline)
            if entry[0] == "rc" and primary_channels(entry) == PRIMARY_NEUTRAL
        )
        disarm_index = self.owner.timeline.index(("arm_request", False))
        self.assertLess(neutral_index, disarm_index)
        self.owner.advance(0.5)
        disarm_index = self.owner.timeline.index(("arm_request", False))
        self.assertFalse(
            any(
                entry[0] == "rc" and rc3_of(entry) == ARM_LOW_RC3_PWM
                for entry in self.owner.timeline[disarm_index + 1 :]
            )
        )

    def test_pilot_frames_cannot_overwrite_low_prearm_window(self) -> None:
        self.owner.arm(True)
        before = len([entry for entry in self.owner.timeline if entry[0] == "rc"])
        self.owner.publish_rc_override(
            yaw=0.5,
            heave=0.5,
            forward=0.5,
            lateral=0.5,
        )
        after = len([entry for entry in self.owner.timeline if entry[0] == "rc"])
        self.assertEqual(after, before)

        self.owner.publish_manual_control(
            yaw=0.5,
            heave=0.5,
            forward=0.5,
            lateral=0.5,
        )
        self.assertFalse(any(entry[0] == "manual" for entry in self.owner.timeline))

    def test_already_armed_never_publishes_low_rc3(self) -> None:
        self.owner._snapshot.armed = True
        self.owner.arm(True)
        self.assertFalse(
            any(
                entry[0] == "rc" and rc3_of(entry) == ARM_LOW_RC3_PWM
                for entry in self.owner.timeline
            )
        )
        self.assertFalse(any(entry[0] == "arm_request" for entry in self.owner.timeline))

    def test_no_simulator_or_rc_options_bypass_in_arm_policy(self) -> None:
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (
                CURRENT / "gui" / "node_arm_commands.py",
                CURRENT / "gui" / "node_arm_rc_sequence.py",
            )
        ).lower()
        self.assertNotIn("rc_options", sources)
        self.assertNotIn("mujoco", sources)

    def test_tk_and_web_arm_controls_share_uuv_gui_node_arm(self) -> None:
        tk_source = (CURRENT / "gui" / "layout_control_modes.py").read_text(
            encoding="utf-8"
        )
        web_source = (CURRENT / "gui" / "web_app.py").read_text(encoding="utf-8")
        self.assertIn("owner.node.arm(True)", tk_source)
        self.assertIn("owner.node.arm(False)", tk_source)
        self.assertIn("self.node.arm(bool(value))", web_source)


if __name__ == "__main__":
    unittest.main(verbosity=2)

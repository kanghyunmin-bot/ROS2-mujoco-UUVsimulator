#!/usr/bin/env python3
"""Process-free fakes for the web GUI/pinger RC ownership hand-off."""

from __future__ import annotations

import queue
import sys
import threading
import time
from pathlib import Path
from types import MethodType, SimpleNamespace

RUNTIME_ROOT = Path(__file__).resolve().parents[1]
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))

from gui import node_init_publishers, node_rc_override_publishers
from gui.web_app import WebGuiController


class _Publisher:
    def __init__(self, events: list[str]) -> None:
        self.events = events

    def publish(self, _message) -> None:
        self.events.append("publish")


class _BlockingPublisher(_Publisher):
    def __init__(self, events: list[str]) -> None:
        super().__init__(events)
        self.started = threading.Event()
        self.allowed_to_finish = threading.Event()

    def publish(self, _message) -> None:
        self.events.append("publish_begin")
        self.started.set()
        assert self.allowed_to_finish.wait(timeout=1.0)
        self.events.append("publish_end")


class _PublisherNode:
    def __init__(self) -> None:
        self.events: list[str] = []
        self._rc_override_publisher_lock = threading.RLock()
        self._rc_override_publisher_suspended = False
        self._rc_override_topic = "/mavros/rc/override"
        self._rc_override_burst_count = 1
        self._rc_override_pub = _Publisher(self.events)
        self.publish_rc_release = MethodType(
            node_rc_override_publishers.publish_rc_release,
            self,
        )

    def destroy_publisher(self, _publisher) -> bool:
        self.events.append("destroy")
        return True

    def create_publisher(self, _message_type, topic: str, qos: int):
        assert topic == "/mavros/rc/override"
        assert qos == 3
        self.events.append("create")
        return _Publisher(self.events)


class _WebNode:
    def __init__(
        self,
        events: list[str],
        *,
        armed: bool = False,
        camera_enabled: bool = False,
    ) -> None:
        self.events = events
        self.armed = armed
        self.camera_enabled = camera_enabled

    def push_event(self, _text: str) -> None:
        return

    def suspend_rc_override_publisher(self) -> bool:
        self.events.append("suspend")
        return True

    def restore_rc_override_publisher(self) -> bool:
        self.events.append("restore")
        return True

    def stereo_camera_status(self) -> dict[str, bool]:
        return {"enabled": self.camera_enabled}

    def set_stereo_camera_enabled(self, enabled: bool) -> None:
        self.camera_enabled = bool(enabled)
        self.events.append(f"camera:{self.camera_enabled}")

    def set_stereo_camera_display_mode(self, mode: str) -> None:
        self.events.append(f"camera_mode:{mode}")

    def probe_backend(self) -> None:
        return

    def snapshot(self):
        return SimpleNamespace(armed=self.armed)


class _Processes:
    def __init__(
        self,
        events: list[str],
        *,
        sim_available: bool = True,
        vision_running: bool = False,
    ) -> None:
        self.events = events
        self.sim_available = sim_available
        self.vision_running = vision_running
        self.pinger_running = False

    def simulation_runtime_available(self) -> bool:
        return self.sim_available

    def start_sim_stack(self, *, purpose: str | None = None) -> dict[str, object]:
        self.events.append(f"sim_start:{purpose}")
        self.sim_available = True
        return {"status": "sim: starting", "purpose": purpose}

    def start_pinger_homing(self, _values) -> dict[str, object]:
        self.events.append("process_start")
        self.pinger_running = True
        return {"status": "started", "running": True}

    def pinger_homing_running(self) -> bool:
        return self.pinger_running

    def stop_pinger_homing(self) -> dict[str, object]:
        self.events.append("process_stop")
        self.pinger_running = False
        return {"status": "stopped", "running": False}

    def vision_preview_running(self) -> bool:
        return self.vision_running

    def stop_vision_processing(self) -> dict[str, object]:
        self.events.append("vision_stop")
        self.vision_running = False
        return {"status": "vision: stopped", "running": False}

    def start_vision_processing(self) -> dict[str, object]:
        self.events.append("vision_start")
        self.vision_running = True
        return {"status": "vision: running", "running": True}


def _controller(events: list[str]) -> WebGuiController:
    controller = WebGuiController.__new__(WebGuiController)
    controller.node = _WebNode(events)
    controller.processes = _Processes(events)
    controller._commands = queue.Queue()
    controller._pinger_rc_handoff_lock = threading.RLock()
    controller._pinger_activation_lock = threading.Lock()
    controller._pinger_activation_generation = 0
    controller._pinger_auto_arm_owned = False
    controller._pinger_auto_arm_candidate_generation = None
    controller._pinger_auto_arm_request_issued_generation = None
    controller._pinger_was_running = False
    controller._pinger_exit_observed_wall = None
    controller._restore_camera_after_pinger = False
    controller._restore_vision_after_pinger = False
    controller.release_rc = lambda: events.append("release_rc")
    controller.enqueue = lambda label, _fn: events.append(f"enqueue:{label}")
    controller._schedule_pinger_activation = lambda **kwargs: events.append(
        f"activate:{kwargs.get('mode')}:{kwargs.get('auto_arm')}"
    )
    controller._cancel_pinger_activation = lambda: None
    return controller


def check_publisher_lifecycle() -> None:
    node_init_publishers.HAVE_MAVROS_MSGS = True
    node = _PublisherNode()
    assert node_init_publishers.suspend_rc_override_publisher(node)
    assert node.events == ["publish", "destroy"], node.events
    assert node._rc_override_pub is None
    assert node._rc_override_publisher_suspended
    assert node_init_publishers.suspend_rc_override_publisher(node)
    assert node.events == ["publish", "destroy"], node.events

    assert node_init_publishers.restore_rc_override_publisher(node)
    assert node.events == ["publish", "destroy", "create"], node.events
    assert node._rc_override_pub is not None
    assert not node._rc_override_publisher_suspended


def check_publish_destroy_serialization() -> None:
    node_init_publishers.HAVE_MAVROS_MSGS = True
    node = _PublisherNode()
    publisher = _BlockingPublisher(node.events)
    node._rc_override_pub = publisher
    publishing = threading.Thread(target=node.publish_rc_release)
    publishing.start()
    assert publisher.started.wait(timeout=1.0)

    suspending = threading.Thread(
        target=node_init_publishers.suspend_rc_override_publisher,
        args=(node,),
    )
    suspending.start()
    assert "destroy" not in node.events
    publisher.allowed_to_finish.set()
    publishing.join(timeout=1.0)
    suspending.join(timeout=1.0)
    assert not publishing.is_alive()
    assert not suspending.is_alive()
    assert node.events == ["publish_begin", "publish_end", "publish_begin", "publish_end", "destroy"], node.events


def check_web_start_stop_order() -> None:
    events: list[str] = []
    controller = _controller(events)
    controller.processes = _Processes(events, vision_running=True)
    result = controller.start_pinger_homing({})
    assert result["running"] is True
    assert "activate:ALT_HOLD:False" in events, events
    assert events.index("release_rc") < events.index("suspend") < events.index("process_start"), events
    assert events.index("vision_stop") < events.index("process_start"), events

    events.clear()
    controller.stop_pinger_homing()
    assert events.index("process_stop") < events.index("restore") < events.index("release_rc"), events
    assert events.index("process_stop") < events.index("vision_start"), events
    assert events.index("vision_start") < events.index("camera_mode:vision"), events


def check_auto_arm_ownership() -> None:
    disarmed_events: list[str] = []
    disarmed_controller = _controller(disarmed_events)
    result = disarmed_controller.start_pinger_homing({"auto_arm": True})
    assert result["running"] is True
    assert not disarmed_controller._pinger_auto_arm_owned
    # Auto-arm ownership is not granted at Start.  It requires both an arm
    # request issued by this generation and a later armed confirmation.
    disarmed_controller._pinger_activation_generation = 7
    disarmed_controller._pinger_auto_arm_candidate_generation = 7
    assert not disarmed_controller._claim_pinger_auto_arm_after_confirmation(7)
    disarmed_controller._pinger_auto_arm_request_issued_generation = 7
    assert disarmed_controller._claim_pinger_auto_arm_after_confirmation(7)
    assert disarmed_controller._pinger_auto_arm_owned
    disarmed_controller.stop_pinger_homing()
    assert disarmed_events.index("release_rc") < disarmed_events.index(
        "enqueue:pinger_homing_auto_disarm"
    ), disarmed_events

    armed_events: list[str] = []
    armed_controller = _controller(armed_events)
    armed_controller.node = _WebNode(armed_events, armed=True)
    result = armed_controller.start_pinger_homing({"auto_arm": True})
    assert result["running"] is True
    assert not armed_controller._pinger_auto_arm_owned
    armed_controller.stop_pinger_homing()
    assert "enqueue:pinger_homing_auto_disarm" not in armed_events, armed_events


def check_repeated_start_is_idempotent() -> None:
    events: list[str] = []
    controller = _controller(events)
    controller.node = _WebNode(events, camera_enabled=True)
    controller.processes = _Processes(events, vision_running=True)

    first = controller.start_pinger_homing({"auto_arm": True})
    assert first["running"] is True
    saved = (
        controller._restore_camera_after_pinger,
        controller._restore_vision_after_pinger,
        controller._pinger_auto_arm_owned,
    )
    event_count = len(events)

    second = controller.start_pinger_homing({"auto_arm": True})
    assert second == {
        "status": "pinger homing: already running",
        "running": True,
        "idempotent": True,
    }, second
    assert len(events) == event_count, events
    assert events.count("process_start") == 1, events
    assert (
        controller._restore_camera_after_pinger,
        controller._restore_vision_after_pinger,
        controller._pinger_auto_arm_owned,
    ) == saved


def check_web_pinger_start_from_stopped_sim_uses_pinger_purpose() -> None:
    events: list[str] = []
    controller = _controller(events)
    controller.processes = _Processes(events, sim_available=False)
    result = controller.start_pinger_homing({})
    assert result["running"] is True, result
    assert "activate:ALT_HOLD:False" in events, events
    assert result["sim"]["purpose"] == "pinger_homing", result
    assert events[0] == "sim_start:pinger_homing", events
    assert events.index("sim_start:pinger_homing") < events.index("release_rc"), events


def check_unexpected_exit_order() -> None:
    events: list[str] = []
    controller = _controller(events)
    controller._pinger_was_running = True
    controller._handle_pinger_process_state({"pinger_homing_running": False})
    assert events == [], events
    controller._pinger_exit_observed_wall = time.monotonic() - 2.0
    controller._handle_pinger_process_state({"pinger_homing_running": False})
    assert events == ["restore", "release_rc"], events

    owned_events: list[str] = []
    owned_controller = _controller(owned_events)
    owned_controller._pinger_was_running = True
    owned_controller._pinger_auto_arm_owned = True
    owned_controller._pinger_exit_observed_wall = time.monotonic() - 2.0
    owned_controller._handle_pinger_process_state({"pinger_homing_running": False})
    assert owned_events == [
        "restore",
        "release_rc",
        "enqueue:pinger_homing_auto_disarm",
    ], owned_events


def main() -> int:
    check_publisher_lifecycle()
    check_publish_destroy_serialization()
    check_web_start_stop_order()
    check_auto_arm_ownership()
    check_repeated_start_is_idempotent()
    check_web_pinger_start_from_stopped_sim_uses_pinger_purpose()
    check_unexpected_exit_order()
    print("gui_pinger_rc_handoff=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

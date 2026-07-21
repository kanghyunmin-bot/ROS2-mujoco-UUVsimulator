#!/usr/bin/env python3
"""Focused regressions for web GUI ownership, launch, and idle-load contracts."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from threading import Event, Thread
from unittest.mock import patch


CURRENT_DIR = Path(__file__).resolve().parents[1]
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from gui.models import TelemetrySnapshot  # noqa: E402
from gui.web_app import UuvWebHandler, WebGuiController  # noqa: E402
from gui.web_process_manager import WebProcessManager  # noqa: E402


class _Node:
    def __init__(self) -> None:
        self.events: list[str] = []
        self.rc_actions: list[str] = []
        self.rc_frames: list[dict[str, float]] = []
        self.camera_modes: list[str] = []
        self.command_actions: list[str] = []
        self.timeline: list[str] = []
        self.armed = False

    def push_event(self, text: str) -> None:
        self.events.append(str(text))

    def probe_backend(self) -> None:
        self.command_actions.append("probe")
        return None

    def arm(self, value: bool) -> None:
        self.command_actions.append(f"arm:{bool(value)}")
        self.timeline.append(f"arm:{bool(value)}")

    def set_mode(self, mode: str) -> None:
        self.command_actions.append(f"mode:{mode}")

    def snapshot(self) -> TelemetrySnapshot:
        return TelemetrySnapshot(armed=self.armed)

    def backend_label(self) -> str:
        return "test"

    def rc_mapping_summary(self) -> str:
        return "test-map"

    def control_readiness(self, _snap: TelemetrySnapshot) -> tuple[str, str]:
        return ("WAIT: test", "")

    def vehicle_info_supported(self) -> bool:
        return False

    def stereo_camera_status(self) -> dict[str, object]:
        return {"enabled": False, "display_mode": "raw"}

    def set_stereo_camera_display_mode(self, mode: str) -> None:
        self.camera_modes.append(mode)

    def set_stereo_camera_enabled(self, _enabled: bool) -> None:
        return None

    def restore_rc_override_publisher(self) -> bool:
        self.timeline.append("restore_rc_publisher")
        return True

    def suspend_rc_override_publisher(self) -> bool:
        self.timeline.append("suspend_rc_publisher")
        return True

    def publish_rc_override(self, **axes: float) -> None:
        frame = {key: float(value) for key, value in axes.items()}
        self.rc_frames.append(frame)
        self.rc_actions.append(
            "neutral" if all(abs(value) <= 1.0e-12 for value in frame.values()) else "active"
        )
        self.timeline.append(self.rc_actions[-1])

    def publish_rc_release(self) -> None:
        self.rc_actions.append("release")
        self.timeline.append("release")

    def request_initial_depth_release_when_armed(self, _reason: str) -> None:
        return None


class _EmptyStatus:
    def status_payload(self) -> dict[str, object]:
        return {}


class _LifecycleStatus(_EmptyStatus):
    def stop(self) -> dict[str, object]:
        return {"status": "stopped"}


class _ControllerProcesses:
    def __init__(self) -> None:
        self.status_calls = 0
        self.vision_starts = 0
        self.pinger_running = False
        self.stop_all_calls = 0

    def status_payload(self) -> dict[str, object]:
        self.status_calls += 1
        return {
            "sim_stack_status": "sim: stopped",
            "ros_pkg_status": "mavros: stopped",
            "rviz_status": "rviz: stopped",
            "ping360_view_status": "ping360 view: closed",
            "pinger_homing_status": "pinger homing: stopped",
            "mission_status": "mission: stopped",
            "pinger_homing_running": False,
            "mission_monitor": {},
            "camera_config": {"preset": "test"},
        }

    def stop_pinger_homing(self) -> dict[str, object]:
        self.pinger_running = False
        return {"status": "pinger homing: stopped", "running": False}

    def start_vision_processing(self) -> dict[str, object]:
        self.vision_starts += 1
        return {"status": "vision: starting", "running": True}

    def stop_vision_processing(self) -> dict[str, object]:
        return {"status": "vision: stopped", "running": False}

    def mission_running(self) -> bool:
        return False

    def pinger_homing_running(self) -> bool:
        return self.pinger_running

    def stop_all(self) -> None:
        self.stop_all_calls += 1
        self.pinger_running = False


class WebGuiRuntimeContractsTest(unittest.TestCase):
    def _controller(self) -> tuple[WebGuiController, _Node, _ControllerProcesses]:
        node = _Node()
        controller = WebGuiController(node)
        processes = _ControllerProcesses()
        controller.processes = processes
        controller.replay = _EmptyStatus()
        controller.tools = _EmptyStatus()
        return controller, node, processes

    def test_pinger_stop_finishes_with_mavros_release(self) -> None:
        controller, node, _processes = self._controller()

        controller.stop_pinger_homing()
        controller._drain_commands()

        self.assertEqual(node.rc_actions, ["neutral", "release"])
        self.assertEqual(node.rc_actions[-1], "release")

    def test_pinger_stop_disarms_only_gui_owned_auto_arm_after_rc_release(self) -> None:
        controller, node, _processes = self._controller()
        controller._pinger_auto_arm_owned = True

        controller.stop_pinger_homing()
        controller._drain_commands()

        self.assertEqual(node.rc_actions, ["neutral", "release"])
        self.assertEqual(node.command_actions, ["probe", "arm:False"])
        self.assertFalse(controller._pinger_auto_arm_owned)
        self.assertTrue(any("auto disarm queued" in event for event in node.events))

        # Ownership is consumed once, so a repeated stop cannot disarm an
        # operator-armed vehicle or send a duplicate disarm request.
        controller.stop_pinger_homing()
        controller._drain_commands()
        self.assertEqual(node.command_actions.count("arm:False"), 1)

    def test_auto_arm_ownership_requires_this_gui_request_and_confirmation(self) -> None:
        controller, node, _processes = self._controller()

        # An external arm that appears before our queued command executes is
        # preserved: no GUI arm request is issued and no ownership is claimed.
        controller._pinger_activation_generation = 10
        controller._pinger_auto_arm_candidate_generation = 10
        node.armed = True
        controller._enqueue_pinger_auto_arm_request(10)
        controller._drain_commands()
        self.assertNotIn("arm:True", node.command_actions)
        self.assertIsNone(controller._pinger_auto_arm_request_issued_generation)
        self.assertFalse(controller._claim_pinger_auto_arm_after_confirmation(10))
        self.assertFalse(controller._pinger_auto_arm_owned)

        # A request actually issued while disarmed is still not ownership.
        # Ownership begins only when a later MAVROS state confirms armed.
        controller._pinger_activation_generation = 11
        controller._pinger_auto_arm_candidate_generation = 11
        node.armed = False
        controller._enqueue_pinger_auto_arm_request(11)
        controller._drain_commands()
        self.assertIn("arm:True", node.command_actions)
        self.assertEqual(controller._pinger_auto_arm_request_issued_generation, 11)
        self.assertFalse(controller._pinger_auto_arm_owned)
        node.armed = True
        self.assertTrue(controller._claim_pinger_auto_arm_after_confirmation(11))
        self.assertTrue(controller._pinger_auto_arm_owned)

    def test_graceful_shutdown_releases_then_disarms_only_owned_arm(self) -> None:
        controller, node, processes = self._controller()
        controller.replay = _LifecycleStatus()
        controller._pinger_was_running = True
        controller._pinger_auto_arm_owned = True
        processes.pinger_running = True
        original_stop_all = processes.stop_all

        def tracked_stop_all() -> None:
            node.timeline.append("stop_all")
            original_stop_all()

        processes.stop_all = tracked_stop_all  # type: ignore[method-assign]

        controller.stop()

        self.assertEqual(processes.stop_all_calls, 1)
        self.assertLess(node.timeline.index("release"), node.timeline.index("arm:False"))
        self.assertLess(node.timeline.index("arm:False"), node.timeline.index("stop_all"))
        self.assertNotIn("mode:", " ".join(node.command_actions))
        self.assertFalse(controller._pinger_auto_arm_owned)

        # Idempotent shutdown cannot repeat release/disarm, and a separate
        # non-owned session never disarms an operator-owned arm.
        controller.stop()
        self.assertEqual(node.command_actions.count("arm:False"), 1)

        external_controller, external_node, external_processes = self._controller()
        external_controller.replay = _LifecycleStatus()
        external_processes.pinger_running = True
        external_controller._pinger_was_running = True
        external_controller.stop()
        self.assertNotIn("arm:False", external_node.command_actions)

    def test_http_arm_and_mode_commands_do_not_probe_on_request_thread(self) -> None:
        controller, node, _processes = self._controller()
        handler = object.__new__(UuvWebHandler)
        handler.controller = controller
        handler.path = "/api/command"
        probe_entered = Event()
        allow_probe = Event()

        def blocking_probe() -> None:
            probe_entered.set()
            self.assertTrue(allow_probe.wait(timeout=1.0))
            node.command_actions.append("probe")

        node.probe_backend = blocking_probe  # type: ignore[method-assign]

        arm_result = handler._handle_command({"command": "arm", "value": True})
        mode_result = handler._handle_command({"command": "mode", "mode": "STABILIZE"})

        self.assertEqual(arm_result, {"command": "arm", "armed": True})
        self.assertEqual(mode_result, {"command": "mode", "mode": "STABILIZE"})
        self.assertFalse(probe_entered.is_set(), "HTTP thread must only enqueue ROS work")

        spin = Thread(target=controller._drain_commands)
        spin.start()
        self.assertTrue(probe_entered.wait(timeout=1.0))
        allow_probe.set()
        spin.join(timeout=1.0)
        self.assertFalse(spin.is_alive())
        self.assertEqual(
            node.command_actions,
            ["probe", "arm:True", "probe", "mode:STABILIZE"],
        )

    def test_out_of_order_rc_frame_cannot_overwrite_newer_axes(self) -> None:
        controller, _node, _processes = self._controller()
        handler = object.__new__(UuvWebHandler)
        handler.controller = controller
        handler.path = "/api/rc"

        newest = handler._handle_command(
            {
                "enabled": True,
                "axes": {"forward": 0.7, "lateral": 0.0, "heave": 0.0, "yaw": 0.1},
                "client_id": "browser-a",
                "seq": 2,
            }
        )
        stale = handler._handle_command(
            {
                "enabled": True,
                "axes": {"forward": -0.8, "lateral": 0.0, "heave": 0.0, "yaw": -0.2},
                "client_id": "browser-a",
                "seq": 1,
            }
        )

        self.assertTrue(newest["accepted"])
        self.assertFalse(stale["accepted"])
        self.assertEqual(controller._axes["forward"], 0.7)
        self.assertEqual(controller._axes["yaw"], 0.1)

    def test_pagehide_release_rejects_late_active_frame(self) -> None:
        controller, node, _processes = self._controller()
        self.assertTrue(
            controller.set_rc(
                enabled=True,
                axes={"forward": 0.6, "lateral": 0.0, "heave": 0.0, "yaw": 0.0},
                client_id="browser-a",
                sequence=10,
            )
        )

        self.assertTrue(controller.release_rc(client_id="browser-a", sequence=12))
        self.assertFalse(
            controller.set_rc(
                enabled=True,
                axes={"forward": 0.9, "lateral": 0.0, "heave": 0.0, "yaw": 0.0},
                client_id="browser-a",
                sequence=11,
            )
        )
        controller._drain_commands()

        self.assertFalse(controller._control_enabled)
        self.assertEqual(node.rc_actions, ["neutral", "release"])

    def test_network_loss_watchdog_releases_once_with_final_release_frame(self) -> None:
        controller, node, _processes = self._controller()
        self.assertTrue(
            controller.set_rc(
                enabled=True,
                axes={"forward": 0.5, "lateral": 0.0, "heave": 0.0, "yaw": 0.0},
                client_id="browser-a",
                sequence=1,
            )
        )
        controller._drain_commands()
        self.assertEqual(node.rc_actions, ["active"])

        timeout_wall = controller._last_rc_receive_wall + controller._rc_watchdog_timeout_s + 0.01
        self.assertTrue(controller._check_rc_watchdog(timeout_wall))
        self.assertFalse(controller._check_rc_watchdog(timeout_wall + 1.0))
        controller._drain_commands()

        self.assertFalse(controller._control_enabled)
        self.assertEqual(node.rc_actions, ["active", "neutral", "release"])
        self.assertEqual(node.rc_actions.count("release"), 1)
        self.assertTrue(any("watchdog released stale input" in event for event in node.events))

        # A pagehide beacon delayed beyond the watchdog must advance the
        # sequence fence without emitting a second release frame.
        self.assertTrue(controller.release_rc(client_id="browser-a", sequence=2))
        controller._drain_commands()
        self.assertEqual(node.rc_actions.count("release"), 1)

    def test_status_collects_process_snapshot_once(self) -> None:
        controller, _node, processes = self._controller()

        payload = controller.status_payload()

        self.assertEqual(processes.status_calls, 1)
        self.assertEqual(payload["camera_config"], {"preset": "test"})

    def test_vision_preview_is_idle_by_default_and_explicit_when_enabled(self) -> None:
        controller, node, processes = self._controller()
        controller._spin_loop = lambda: None

        with patch.dict(os.environ, {"UUV_GUI_VISION_AUTO_START": "0"}):
            controller.start()
            controller._spin_thread.join(timeout=1.0)
        self.assertEqual(processes.vision_starts, 0)
        self.assertEqual(node.camera_modes[-1], "raw")

        controller.set_vision_processing_enabled(True)
        self.assertEqual(processes.vision_starts, 1)
        self.assertEqual(node.camera_modes[-1], "vision")

    def test_preview_launch_uses_current_vision_package_contract(self) -> None:
        manager = WebProcessManager(_Node())
        captured: list[list[str]] = []

        def capture_start(**kwargs: object) -> dict[str, object]:
            captured.append(list(kwargs["cmd"]))
            return {"status": "captured"}

        manager._start_plain_process = capture_start  # type: ignore[method-assign]
        manager.start_vision_processing()
        command = "\n".join(captured[-1])

        self.assertIn("OMP_NUM_THREADS=1", command)
        self.assertIn("MKL_NUM_THREADS=1", command)
        self.assertIn("OPENBLAS_NUM_THREADS=1", command)
        self.assertIn("NUMEXPR_NUM_THREADS=1", command)
        self.assertIn("annotated_image_topic:=/vision/buoy/image_annotated/compressed", command)
        self.assertIn("publish_annotated_image:=true", command)
        self.assertIn("imgsz:=512", command)
        self.assertIn("publish_per_class:=false", command)

    def test_frontend_debounces_and_disables_repeated_pinger_start(self) -> None:
        script = (CURRENT_DIR / "gui" / "web_static" / "app.js").read_text(
            encoding="utf-8"
        )
        index = (CURRENT_DIR / "gui" / "web_static" / "index.html").read_text(
            encoding="utf-8"
        )
        self.assertIn("pingerStartPending", script)
        self.assertIn(
            'pingerStartButton.disabled = pingerRunning || state.pingerStartPending',
            script,
        )
        self.assertIn(
            'if (state.pingerStartPending || $("pingerHomingStartBtn").disabled)',
            script,
        )
        self.assertIn(
            'addEventListener("click", startPingerHoming)',
            script,
        )
        self.assertIn(
            'no_odom_probe_pwm_delta: numericValue("pingerHomingProbePwmDelta", 90)',
            script,
        )
        self.assertIn(
            'no_odom_approach_pwm_delta: numericValue("pingerHomingApproachPwmDelta", 200)',
            script,
        )
        self.assertIn(
            'no_odom_forward_duration_s: numericValue("pingerHomingApproachDuration", 40)',
            script,
        )
        self.assertIn(
            '<option value="no_odom_phase" selected>Phase ABBA (no odometry)</option>',
            index,
        )
        self.assertNotIn('id="pingerHomingUseYolo"', index)
        self.assertNotIn('id="pingerHomingYoloRange"', index)
        self.assertNotIn('<option value="MANUAL">MANUAL</option>', index)

    def test_frontend_accepts_a_physical_browser_gamepad_for_pilot_input(self) -> None:
        script = (CURRENT_DIR / "gui" / "web_static" / "app.js").read_text(
            encoding="utf-8"
        )
        index = (CURRENT_DIR / "gui" / "web_static" / "index.html").read_text(
            encoding="utf-8"
        )
        self.assertIn("navigator.getGamepads", script)
        self.assertIn("function physicalGamepadAxes", script)
        self.assertIn("function pollPhysicalGamepad", script)
        self.assertIn('addEventListener("gamepadconnected"', script)
        self.assertIn("setInterval(pollPhysicalGamepad, GAMEPAD_POLL_MS)", script)
        self.assertIn('id="gamepadEnabled"', index)
        self.assertIn('id="gamepadStatus"', index)

    def test_mission_selects_real_and_sim_pose_contracts(self) -> None:
        manager = WebProcessManager(_Node())
        manager._clear_mission_status_file = lambda: None  # type: ignore[method-assign]
        captured: list[list[str]] = []

        def capture_start(**kwargs: object) -> dict[str, object]:
            captured.append(list(kwargs["cmd"]))
            return {"status": "captured"}

        manager._start_plain_process = capture_start  # type: ignore[method-assign]

        manager._simulation_runtime_available = lambda: False  # type: ignore[method-assign]
        manager.start_ground_truth_mission(
            {"dry_run": True, "transport": "command_override"}
        )
        real_command = "\n".join(captured[-1])
        self.assertIn("kmu26_vision_mission_fsm", real_command)
        self.assertIn("pose_topic:=/odometry/filtered", real_command)
        self.assertIn("transport:=command_override", real_command)

        manager._simulation_runtime_available = lambda: True  # type: ignore[method-assign]
        manager.start_ground_truth_mission({"dry_run": True, "transport": "rc_override"})
        sim_command = "\n".join(captured[-1])
        self.assertIn("pose_topic:=/sim/odom", sim_command)
        self.assertIn("transport:=rc_override", sim_command)


if __name__ == "__main__":
    unittest.main()

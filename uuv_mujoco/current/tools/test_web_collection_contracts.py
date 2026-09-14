#!/usr/bin/env python3
"""Regressions for collection sensor settings and session provenance isolation."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gui.sim_stack_launch_command import camera_config_from_launch_command, camera_optics_environment, normalize_camera_config
from gui.web_app import UuvWebHandler
from gui.web_process_manager import WebProcessManager
from gui.web_recorder import WebRecorder


class CollectionCameraContracts(unittest.TestCase):
    def recorder(self, camera=None):
        recorder = WebRecorder.__new__(WebRecorder)
        recorder.lock = threading.RLock()
        recorder.closed = False
        recorder.session = ""
        recorder.process = None
        recorder.status = {}
        recorder.received = 0.0
        recorder.future = None
        recorder.status_future = None
        recorder.message = ""
        recorder.processes = SimpleNamespace(camera_config_payload=lambda: camera or {"hz": 4.0})
        recorder.clients = {key: Mock(service_is_ready=lambda: False) for key in ("get_status", "start_episode")}
        return recorder

    def test_lightweight_vla_rate_keeps_default_unchanged(self):
        self.assertEqual(normalize_camera_config()["hz"], 4.0)
        config = normalize_camera_config({"preset_id": "vla_lite"}, strict_preset=True)
        self.assertEqual((config["width"], config["height"], config["hz"]), (640, 360, 15.0))

    def test_prepare_rejects_known_slow_or_disabled_sensor_before_creating_files(self):
        for camera in ({"hz": 4.0}, {"hz": 15.0, "launched": {"hz": 4.0, "enabled": True}},
                       {"hz": 15.0, "launched": {"hz": 15.0, "enabled": False}}):
            with self.subTest(camera=camera), TemporaryDirectory() as directory:
                with patch("gui.web_recorder.APP_ROOT", Path(directory)), patch(
                    "gui.web_recorder.Path.mkdir", side_effect=AssertionError("Invalid camera created a session directory")
                ):
                    with self.assertRaisesRegex(ValueError, "10|카메라"):
                        self.recorder(camera).prepare("Approach buoy", "STABILIZE")
                self.assertEqual(list(Path(directory).iterdir()), [])

    def test_idle_owned_session_blocks_reconfiguration_even_without_status(self):
        recorder = self.recorder()
        recorder.session = "sim_owned"
        recorder.process = Mock(poll=lambda: None)
        controller = SimpleNamespace(recorder=recorder, configure_camera=Mock(return_value={}),
                                     reset_sim_stack=Mock(return_value={}), start_sim_stack=Mock(return_value={}),
                                     save_tool_file=Mock(return_value={}), save_course_layout=Mock(return_value={}),
                                     apply_physics_params=Mock(return_value={}))
        handler = UuvWebHandler.__new__(UuvWebHandler)
        handler.controller = controller
        handler.path = "/api/command"
        for command in ("camera_config", "stack_reset", "course_save", "physics_apply", "stack_start", "tool_save"):
            with self.subTest(command=command), self.assertRaisesRegex(ValueError, "세션"):
                handler._handle_command({"command": command, "values": {"preset_id": "vla_lite"}})
        controller.configure_camera.assert_not_called()
        controller.reset_sim_stack.assert_not_called()

    def test_failed_collector_retains_configuration_lock_until_session_close(self):
        recorder = self.recorder()
        recorder.session = "sim_owned"
        recorder.process = Mock(poll=lambda: 1)
        self.assertTrue(recorder.payload()["configuration_locked"])
        recorder.close_session()
        self.assertFalse(recorder.payload()["configuration_locked"])

    def test_alive_collector_cannot_unlock_before_process_exit(self):
        recorder = self.recorder()
        recorder.session = "sim_owned"
        recorder.process = Mock(poll=lambda: None)
        recorder.payload = lambda: {"online": True, "owned": True, "running": True, "busy": False, "active": False}
        recorder.shutdown = Mock()
        with self.assertRaisesRegex(ValueError, "종료 대기"):
            recorder.close_session()
        self.assertEqual(recorder.session, "sim_owned")

    def test_dead_collector_close_cancels_pending_command_before_next_session(self):
        recorder = self.recorder()
        recorder.session = "sim_owned"
        recorder.process = Mock(poll=lambda: 1)
        pending = recorder.future = Mock()
        client = recorder.command_client = Mock()
        recorder.close_session()
        client.remove_pending_request.assert_called_once_with(pending)
        pending.cancel.assert_called_once_with()
        self.assertFalse(recorder.payload()["busy"])
        self.assertFalse(recorder.payload()["configuration_locked"])

    def test_old_command_callback_cannot_clear_next_sessions_command(self):
        recorder = self.recorder()
        current = recorder.future = Mock()
        recorder.received = 42.0
        recorder.message = "new session command pending"
        stale = Mock(result=lambda: SimpleNamespace(success=True, message="old session result"))
        recorder._command_done(stale)
        self.assertIs(recorder.future, current)
        self.assertEqual(recorder.received, 42.0)
        self.assertEqual(recorder.message, "new session command pending")

    def test_queued_sensor_change_cannot_execute_after_session_preparation(self):
        recorder = self.recorder()
        queued = []
        node = SimpleNamespace(publish_ping360_enabled=Mock())
        controller = SimpleNamespace(recorder=recorder, node=node,
                                     enqueue=lambda label, callback: queued.append(callback))
        handler = UuvWebHandler.__new__(UuvWebHandler)
        handler.controller = controller
        handler.path = "/api/command"
        handler._handle_command({"command": "ping360_enabled", "enabled": False})
        self.assertEqual(len(queued), 1)
        recorder.session = "sim_prepared_before_queue_drained"
        with self.assertRaisesRegex(ValueError, "세션"):
            queued.pop()()
        node.publish_ping360_enabled.assert_not_called()
        recorder.session = ""
        handler._handle_command({"command": "ping360_enabled", "enabled": True})
        queued.pop()()
        node.publish_ping360_enabled.assert_called_once_with(True)

    def test_saving_sensor_profile_preserves_optics_and_launched_contract(self):
        manager = WebProcessManager(SimpleNamespace(push_event=lambda _: None))
        manager.configure_camera({"preset_id": "vla_lite", "optics_profile": "pool_lite"})
        manager._active_camera_config = {"hz": 4.0, "enabled": True}
        manager._sim_process = Mock(poll=lambda: None)
        manager.configure_camera({"preset_id": "smooth540"})
        payload = manager.camera_config_payload()
        self.assertEqual(payload["optics_profile"], "pool_lite")
        self.assertEqual(payload["hz"], 10.0)
        self.assertEqual(payload["launched"]["hz"], 4.0)
        self.assertFalse(payload["rate_measured"])

    def test_launch_overrides_take_precedence_over_saved_vla_rate(self):
        selected = normalize_camera_config({"preset_id": "vla_lite", "optics_profile": "pool_lite"})
        launched = camera_config_from_launch_command(selected, ["launch", "--ros2-images", "--ros2-image-width=320",
                                                              "--ros2-image-height", "240", "--ros2-image-hz=4"])
        self.assertEqual((launched["width"], launched["height"], launched["hz"]), (320, 240, 4.0))
        self.assertTrue(launched["enabled"])
        self.assertEqual(launched["optics_profile"], "pool_lite")
        self.assertFalse(camera_config_from_launch_command(selected, ["launch"])["enabled"])
        self.assertEqual(camera_optics_environment({"optics_profile": "inherited"}), {})
        self.assertTrue(camera_optics_environment(selected)["ROS2_UUV_CAMERA_SENSOR_MODEL_CONFIG"].endswith("pool_lite.json"))


if __name__ == "__main__":
    unittest.main()

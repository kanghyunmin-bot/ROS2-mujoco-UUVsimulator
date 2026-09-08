#!/usr/bin/env python3
"""Regression tests for the strict SITL IMU/Bar30 host boundary."""

from __future__ import annotations

import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


CURRENT = Path(__file__).resolve().parents[1]
REPO = CURRENT.parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.ros2_bridge_publish_timing import publish_time_due  # noqa: E402
from bridge.ros2_bridge_runtime_setup import configure_bridge_runtime_state  # noqa: E402
from sim.runtime.ros_bridge_launch_config import ros_bridge_kwargs_from_args  # noqa: E402


def args(**overrides):
    values = {
        "ping360_range_m": 10.0,
        "ping360_num_steps": 1200,
        "ping360_interface": "serial",
        "ping360_gain": 1,
        "ros2_images": False,
        "ros2_image_width": 1280,
        "ros2_image_height": 720,
        "ros2_sensor_hz": 400.0,
        "ros2_image_hz": 30.0,
        "sitl": True,
        "sitl_ip": "127.0.0.1",
        "sitl_port": 9002,
        "sitl_send_port": 9003,
        "sitl_mavlink_endpoint": "",
        "sitl_mavlink_servo_hz": 50.0,
        "sitl_mavlink_target_sysid": 1,
        "sitl_mavlink_target_compid": 1,
        "sitl_mavlink_source_sysid": 255,
        "sitl_mavlink_source_compid": 190,
        "ros2_camera_calib_left": "",
        "ros2_camera_calib_right": "",
        "ros2": True,
        "ros2_real_pkg_compat": True,
        "no_ping360": False,
        "ping360_config": "",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class StrictSitlSensorTransportTest(unittest.TestCase):
    def test_launch_mapping_activates_only_for_strict_sitl(self) -> None:
        strict = ros_bridge_kwargs_from_args(
            args=args(), model=object(), command_callback=lambda *_values: None, cmd_limit=1.0
        )
        non_strict = ros_bridge_kwargs_from_args(
            args=args(ros2_real_pkg_compat=False),
            model=object(),
            command_callback=lambda *_values: None,
            cmd_limit=1.0,
        )
        no_sitl = ros_bridge_kwargs_from_args(
            args=args(sitl=False),
            model=object(),
            command_callback=lambda *_values: None,
            cmd_limit=1.0,
        )
        no_ros = ros_bridge_kwargs_from_args(
            args=args(ros2=False),
            model=object(),
            command_callback=lambda *_values: None,
            cmd_limit=1.0,
        )
        self.assertTrue(strict["strict_sitl_sensor_transport"])
        self.assertFalse(non_strict["strict_sitl_sensor_transport"])
        self.assertFalse(no_sitl["strict_sitl_sensor_transport"])
        self.assertFalse(no_ros["strict_sitl_sensor_transport"])

    def test_bridge_runtime_requires_ros_sitl_and_real_pkg_together(self) -> None:
        def configure(*, enable_ros: bool, enable_sitl: bool, strict: bool):
            bridge = SimpleNamespace()
            with patch.dict(
                os.environ,
                {"ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE": "0"},
            ):
                configure_bridge_runtime_state(
                    bridge,
                    model=None,
                    command_callback=lambda *_values: None,
                    cmd_limit=1.0,
                    publish_images=False,
                    image_width=64,
                    image_height=64,
                    sensor_hz=400.0,
                    image_hz=1.0,
                    camera_calib_left="",
                    camera_calib_right="",
                    enable_sitl=enable_sitl,
                    enable_ros=enable_ros,
                    enable_mavros_surface=False,
                    real_pkg_compat=True,
                    strict_sitl_sensor_transport=strict,
                )
            return bridge

        active = configure(enable_ros=True, enable_sitl=True, strict=True)
        disabled = configure(enable_ros=False, enable_sitl=True, strict=False)
        self.assertTrue(active._strict_sitl_sensor_transport)
        self.assertFalse(disabled._strict_sitl_sensor_transport)
        with self.assertRaises(ValueError):
            configure(enable_ros=False, enable_sitl=True, strict=True)

    def test_backward_clock_reopens_publish_gate_and_rate_schedules(self) -> None:
        bridge = SimpleNamespace(
            last_pub_t=10.0,
            sensor_dt=0.0025,
            _ros_sensor_rate_next_t={"/mavros/imu/data": 10.02},
        )
        self.assertTrue(publish_time_due(bridge, 1.0))
        self.assertEqual(bridge.last_pub_t, 1.0)
        self.assertEqual(bridge._ros_sensor_rate_next_t, {})

    def test_sim_mavros_quarantines_duplicate_sensor_publishers(self) -> None:
        launch_path = REPO / "rospkg/src/kmu26_auv/launch/mavros_apm_sim.launch.py"
        source = launch_path.read_text(encoding="utf-8")
        for topic in ("imu/data_raw", "imu/static_pressure"):
            self.assertIn(f'"/mavros/{topic}"', source)
            self.assertIn(
                f'"/uuv_mujoco/mavros_fcu_passthrough/{topic}"',
                source,
            )
        self.assertNotIn(
            '"/uuv_mujoco/mavros_fcu_passthrough/imu/data"',
            source,
        )
        self.assertIn("MAVROS retains the FCU AHRS owner", source)
        self.assertIn("/uuv_mujoco/mavros_fcu_passthrough/", source)


if __name__ == "__main__":
    unittest.main(verbosity=2)

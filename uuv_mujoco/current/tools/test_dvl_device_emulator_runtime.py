#!/usr/bin/env python3
"""Offline integration tests for the bridge-to-A50 device boundary."""

from __future__ import annotations

import json
import os
import socket
import sys
import time
import unittest
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.dvl_a50_sensor_model import A50SensorConfig, A50SensorModel  # noqa: E402
from bridge.ros2_dvl_device_emulator_runtime import (  # noqa: E402
    ENABLE_ENV,
    close_dvl_device_emulator,
    configure_dvl_device_emulator_runtime,
    forward_dvl_delivery_to_device_emulator,
    forward_dvl_position_to_device_emulator,
    synchronize_dvl_device_commands,
)
from bridge.ros2_dvl_sensor_runtime import (  # noqa: E402
    configure_dvl_sensor_runtime,
)
from bridge.ros2_bridge_servo_api import reset_odometry  # noqa: E402


EMULATOR_ENV_PREFIX = "ROS2_UUV_DVL_DEVICE_EMULATOR_"


class FakeBridge:
    def __init__(self) -> None:
        self._bmj_to_frd = np.eye(3, dtype=np.float64)
        self._dvl_body_frd_to_dvl_frd = np.eye(3, dtype=np.float64)
        self._odom_pos = np.zeros(3, dtype=np.float64)


@contextmanager
def emulator_environment(**overrides: str):
    environment = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith(EMULATOR_ENV_PREFIX)
    }
    environment.update(
        {
            ENABLE_ENV: "1",
            f"{EMULATOR_ENV_PREFIX}HOST": "127.0.0.1",
            f"{EMULATOR_ENV_PREFIX}PORT": "0",
            **overrides,
        }
    )
    with patch.dict(os.environ, environment, clear=True):
        yield


def read_json_lines(client: socket.socket, count: int) -> list[dict[str, object]]:
    deadline = time.monotonic() + 2.0
    buffer = bytearray()
    reports = []
    while len(reports) < count and time.monotonic() < deadline:
        try:
            chunk = client.recv(65_536)
        except TimeoutError:
            continue
        if not chunk:
            break
        buffer.extend(chunk)
        while b"\n" in buffer:
            line, _, remainder = buffer.partition(b"\n")
            buffer[:] = remainder
            if line:
                reports.append(json.loads(line))
    if len(reports) != count:
        raise AssertionError(f"expected {count} reports, received {len(reports)}")
    return reports


class DvlDeviceEmulatorRuntimeTest(unittest.TestCase):
    def test_forwards_velocity_at_10_hz_and_position_at_5_hz(self) -> None:
        bridge = FakeBridge()
        bridge._dvl_sensor_model_enabled = True
        bridge._dvl_sensor_timing_config = SimpleNamespace(
            schedule=SimpleNamespace(rate_hz=10.0)
        )
        model = A50SensorModel(
            A50SensorConfig(
                seed=8,
                white_noise_std_mps=0.0,
                velocity_noise_per_meter_mps=0.0,
                range_noise_std_m=0.0,
                range_noise_fraction=0.0,
            )
        )

        with emulator_environment():
            configure_dvl_device_emulator_runtime(bridge)
            self.addCleanup(close_dvl_device_emulator, bridge)
            client = socket.create_connection(bridge._dvl_device_emulator.address)
            self.addCleanup(client.close)
            client.settimeout(0.1)

            for index in range(5):
                capture_time_s = index * 0.1
                sample = model.sample(
                    (0.2, -0.1, 0.0),
                    2.0,
                    time_of_validity_us=int(capture_time_s * 1_000_000),
                    time_of_transmission_us=int(capture_time_s * 1_000_000),
                )
                delivery = SimpleNamespace(
                    sample=sample,
                    capture_time_s=capture_time_s,
                    arrival_time_s=capture_time_s,
                )
                forward_dvl_delivery_to_device_emulator(bridge, delivery)
                if index % 2 == 0:
                    position = SimpleNamespace(
                        capture_time_s=capture_time_s,
                        arrival_time_s=capture_time_s,
                        report_time_s=capture_time_s,
                        position_local_frd_m=(capture_time_s * 0.2, 0.0, 0.0),
                        position_std_m=0.01,
                        attitude_rpy_deg=(0.0, 0.0, 0.0),
                    )
                    forward_dvl_position_to_device_emulator(bridge, position)

            reports = read_json_lines(client, 8)

        self.assertEqual(
            [report["type"] for report in reports],
            [
                "velocity",
                "position_local",
                "velocity",
                "velocity",
                "position_local",
                "velocity",
                "velocity",
                "position_local",
            ],
        )

    def test_driver_reset_command_resets_measured_dead_reckoning(self) -> None:
        bridge = FakeBridge()
        configure_dvl_sensor_runtime(bridge)
        with emulator_environment():
            configure_dvl_device_emulator_runtime(bridge)
            self.addCleanup(close_dvl_device_emulator, bridge)
            client = socket.create_connection(bridge._dvl_device_emulator.address)
            self.addCleanup(client.close)
            client.settimeout(0.1)
            bridge._dvl_sensor_position_local_frd_m[:] = (1.0, 2.0, 3.0)
            client.sendall(b'{"command":"reset_dead_reckoning"}\n')
            response = read_json_lines(client, 1)[0]
            self.assertTrue(response["success"])

            synchronize_dvl_device_commands(bridge)

        np.testing.assert_allclose(
            bridge._dvl_sensor_position_local_frd_m,
            (0.0, 0.0, 0.0),
            atol=0.0,
        )
        self.assertEqual(bridge._dvl_device_emulator_seen_reset_count, 1)

    def test_local_odometry_reset_purges_pre_reset_device_position(self) -> None:
        bridge = FakeBridge()
        configure_dvl_sensor_runtime(bridge)
        with emulator_environment():
            configure_dvl_device_emulator_runtime(bridge)
            self.addCleanup(close_dvl_device_emulator, bridge)
            position = SimpleNamespace(
                report_time_s=0.0,
                position_local_frd_m=(1.0, 2.0, 3.0),
                position_std_m=0.01,
                attitude_rpy_deg=(0.0, 0.0, 0.0),
            )
            forward_dvl_position_to_device_emulator(bridge, position)
            self.assertEqual(bridge._dvl_device_emulator.queued_message_count, 1)

            reset_odometry(bridge)

            self.assertEqual(bridge._dvl_device_emulator.queued_message_count, 0)
            self.assertEqual(
                bridge._dvl_device_emulator.dead_reckoning_reset_count,
                1,
            )
            self.assertEqual(bridge._dvl_device_emulator_seen_reset_count, 1)
            self.assertEqual(bridge._dvl_sensor_dr_generation, 1)

    def test_disabled_mode_does_not_bind_a_socket(self) -> None:
        bridge = FakeBridge()
        bridge._dvl_sensor_model_enabled = True
        bridge._dvl_sensor_timing_config = SimpleNamespace(
            schedule=SimpleNamespace(rate_hz=10.0)
        )
        clean_environment = {
            key: value
            for key, value in os.environ.items()
            if not key.startswith(EMULATOR_ENV_PREFIX)
        }
        with patch.dict(os.environ, clean_environment, clear=True):
            configure_dvl_device_emulator_runtime(bridge)
        self.assertFalse(bridge._dvl_device_emulator_enabled)
        self.assertIsNone(bridge._dvl_device_emulator)

    def test_strict_real_package_mode_enables_device_boundary_by_default(self) -> None:
        bridge = FakeBridge()
        bridge._real_pkg_compat = True
        bridge._dvl_sensor_model_enabled = True
        bridge._dvl_sensor_timing_config = SimpleNamespace(
            schedule=SimpleNamespace(rate_hz=10.0)
        )
        environment = {
            key: value
            for key, value in os.environ.items()
            if not key.startswith(EMULATOR_ENV_PREFIX)
        }
        environment.update(
            {
                f"{EMULATOR_ENV_PREFIX}HOST": "127.0.0.1",
                f"{EMULATOR_ENV_PREFIX}PORT": "0",
            }
        )
        with patch.dict(os.environ, environment, clear=True):
            configure_dvl_device_emulator_runtime(bridge)
        self.addCleanup(close_dvl_device_emulator, bridge)

        self.assertTrue(bridge._dvl_device_emulator_enabled)
        self.assertIsNotNone(bridge._dvl_device_emulator)
        self.assertTrue(bridge._dvl_device_emulator.is_running)

    def test_strict_real_package_mode_rejects_direct_ros_opt_out(self) -> None:
        bridge = FakeBridge()
        bridge._real_pkg_compat = True
        bridge._dvl_sensor_model_enabled = True
        with emulator_environment(**{ENABLE_ENV: "0"}):
            with self.assertRaisesRegex(ValueError, "sole owner"):
                configure_dvl_device_emulator_runtime(bridge)

    def test_enabled_mode_requires_sensor_model(self) -> None:
        bridge = FakeBridge()
        bridge._dvl_sensor_model_enabled = False
        with emulator_environment():
            with self.assertRaisesRegex(ValueError, "sensor model"):
                configure_dvl_device_emulator_runtime(bridge)


if __name__ == "__main__":
    unittest.main(verbosity=2)

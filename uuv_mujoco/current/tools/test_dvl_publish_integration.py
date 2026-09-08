#!/usr/bin/env python3
"""Regression tests for DVL publish integration and legacy fallback."""

from __future__ import annotations

import json
import os
import socket
import sys
import threading
import time
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.dvl_a50_sensor_model import A50SensorConfig, A50SensorModel  # noqa: E402
from bridge.dvl_a50_tcp_emulator import A50TcpJsonEmulator  # noqa: E402
from bridge.ros2_bridge_context_runtime import initialize_ros2_endpoints  # noqa: E402
from bridge.ros2_bridge_constructor import initialize_ros2_bridge  # noqa: E402
from bridge.ros2_bridge_publish_ros import publish_ros_snapshot  # noqa: E402
from bridge.ros2_dvl_sensor_runtime import configure_dvl_sensor_runtime  # noqa: E402
from bridge.ros2_dvl_device_emulator_runtime import (  # noqa: E402
    close_dvl_device_emulator,
    configure_dvl_device_emulator_runtime,
)
from bridge.ros2_endpoint_misc_publishers import (  # noqa: E402
    create_dvl_compat_publishers,
)
from bridge.ros2_publish_dvl_factories import (  # noqa: E402
    build_dvl_data_msg,
    build_dvl_position_msg,
)
from bridge.ros2_publish_dvl_cache import DvlPublishBuilderCache  # noqa: E402
from bridge.ros2_publish_state import (  # noqa: E402
    _forward_dvl_device_deliveries,
    prepare_ros_publish_state,
)
from bridge.ros2_publish_schedule_dvl import (  # noqa: E402
    schedule_real_dvl_compat_jobs,
)
from bridge.ros2_runtime_safe_publish import safe_publish  # noqa: E402
from bridge.ros2_sitl_sensor_types import Ros2SensorSnapshot  # noqa: E402


class Stamp:
    def __init__(self) -> None:
        self.sec = 0
        self.nanosec = 0


class Header:
    def __init__(self) -> None:
        self.stamp = None
        self.frame_id = ""


class DvldrMsg:
    def __init__(self) -> None:
        self.header = Header()
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.time = 0.0
        self.pos_std = 0.0
        self.type = ""
        self.status = -1
        self.format = ""


class DvlMsg:
    def __init__(self) -> None:
        self.header = Header()
        self.time = 0.0
        self.velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.fom = 0.0
        self.covariance = []
        self.altitude = 0.0
        self.beams = []
        self.velocity_valid = False
        self.status = -1
        self.time_of_validity = 0
        self.time_of_transmission = 0
        self.form = ""


class FakeLegacyBridge:
    def __init__(self) -> None:
        self._bmj_to_flu = np.eye(3, dtype=np.float64)
        self._bmj_to_frd = np.diag((1.0, -1.0, -1.0))
        self._dvl_body_frd_to_dvl_frd = np.eye(3, dtype=np.float64)
        self._odom_pos = np.zeros(3, dtype=np.float64)
        self._last_odom_time = -1.0
        self.sensor_dt = 0.1
        self._static_pressure_source = "external"
        self._internal_pressure_pa = 101_325.0
        self._dvl_device_emulator = None

    @staticmethod
    def _ros_imu_accel_surface(value):
        return np.asarray(value, dtype=np.float64)

    @staticmethod
    def _apply_mavros_setpoint(_position, _rotation) -> None:
        return None


def sensor_snapshot() -> Ros2SensorSnapshot:
    return Ros2SensorSnapshot(
        base_pos_enu=np.zeros(3, dtype=np.float64),
        base_rot_enu=np.eye(3, dtype=np.float64),
        quat_base=np.array((1.0, 0.0, 0.0, 0.0), dtype=np.float64),
        base_vel_enu=np.array((1.0, 0.0, 0.0), dtype=np.float64),
        gyro_bmj=np.zeros(3, dtype=np.float64),
        acc_bmj=np.zeros(3, dtype=np.float64),
        dvl_vel_body_bmj=np.array((1.0, 0.0, 0.0), dtype=np.float64),
        dvl_altitude_m=2.0,
        bar30_pressure_pa=120_000.0,
        ros_depth_m=2.0,
    )


class DvlPublishIntegrationTest(unittest.TestCase):
    def test_authoritative_clock_continues_after_optional_ros_publish_failure(self) -> None:
        class ClockMsg:
            def __init__(self) -> None:
                self.clock = Stamp()

        class RecordingPublisher:
            def __init__(self) -> None:
                self.stamps = []

            def publish(self, message) -> None:
                self.stamps.append((message.clock.sec, message.clock.nanosec))

        class FailingPublisher:
            @staticmethod
            def publish(_message) -> None:
                raise RuntimeError("injected optional publisher failure")

        public_clock = RecordingPublisher()
        private_clock = RecordingPublisher()
        bridge = SimpleNamespace(
            _enable_ros=True,
            _ros_ok=True,
            _ros_error_reported=False,
            Clock=ClockMsg,
            pub_clock=public_clock,
            pub_uuv_mujoco_clock=private_clock,
            _publish_static_context=lambda _stamp, _sim_t: True,
        )
        bridge._safe_publish = lambda publisher, message, label: safe_publish(
            bridge,
            publisher,
            message,
            label,
        )
        bridge._flush_ros_publish_jobs = lambda *_args: bridge._safe_publish(
            FailingPublisher(),
            object(),
            "/optional/failure",
        )

        with patch(
            "bridge.ros2_bridge_publish_ros.prepare_ros_publish_state",
            return_value=object(),
        ):
            publish_ros_snapshot(bridge, object(), 0.0, object())
            self.assertFalse(bridge._ros_ok)
            publish_ros_snapshot(bridge, object(), 0.1, object())
            publish_ros_snapshot(bridge, object(), 0.13, object())

        expected = [(0, 0), (0, 100_000_000), (0, 130_000_000)]
        self.assertEqual(public_clock.stamps, expected)
        self.assertEqual(private_clock.stamps, expected)

    def test_constructor_failure_rolls_back_device_sitl_and_partial_ros(self) -> None:
        class ResourceBridge:
            def __init__(self) -> None:
                self.poll_stop = threading.Event()

            def _start_sitl_poll_thread(self) -> None:
                self._sitl_poll_thread = threading.Thread(
                    target=self.poll_stop.wait,
                    name="constructor-rollback-test",
                    daemon=True,
                )
                self._sitl_poll_thread.start()

            def _stop_sitl_poll_thread(self) -> None:
                self.poll_stop.set()
                if self._sitl_poll_thread is not None:
                    self._sitl_poll_thread.join(timeout=1.0)
                self._sitl_poll_thread = None

            @staticmethod
            def _init_ros() -> None:
                raise RuntimeError("injected partial ROS failure")

        class FakeTransport:
            def __init__(self) -> None:
                self.was_shutdown = False

            def shutdown(self) -> None:
                self.was_shutdown = True

        class FakeExecutor:
            def __init__(self) -> None:
                self.removed = None
                self.was_shutdown = False

            def remove_node(self, node) -> None:
                self.removed = node

            def shutdown(self, *, timeout_sec) -> None:
                self.was_shutdown = timeout_sec == 0.0

        class FakeNode:
            def __init__(self) -> None:
                self.was_destroyed = False

            def destroy_node(self) -> None:
                self.was_destroyed = True

        class FakeContext:
            def __init__(self) -> None:
                self.was_shutdown = False

            def ok(self) -> bool:
                return not self.was_shutdown

            def shutdown(self) -> None:
                self.was_shutdown = True

        bridge = ResourceBridge()
        transport = FakeTransport()
        executor = FakeExecutor()
        node = FakeNode()
        context = FakeContext()
        resources = {}

        def configure_base(owner, **_kwargs) -> None:
            owner._enable_ros = True
            owner.enable_sitl = True
            owner._dvl_device_emulator = None
            owner._ros_spin_stop = threading.Event()
            owner._ros_spin_thread = None
            owner._sitl_poll_thread = None
            owner._sitl_transport_lock = threading.RLock()

        def configure_contracts(owner) -> None:
            emulator = A50TcpJsonEmulator(port=0).start()
            owner._dvl_device_emulator = emulator
            resources["emulator"] = emulator
            resources["address"] = emulator.address

        def configure_ros(owner) -> None:
            owner._executor = executor
            owner.node = node
            owner._ros_context = context

        init = {
            "model": object(),
            "command_callback": lambda *_args: None,
            "cmd_limit": 1.0,
            "publish_images": False,
            "image_width": 64,
            "image_height": 64,
            "sensor_hz": 100.0,
            "image_hz": 10.0,
            "camera_calib_left": "",
            "camera_calib_right": "",
            "enable_sitl": True,
            "enable_ros": True,
            "enable_mavros_surface": False,
            "real_pkg_compat": True,
            "enable_ping360": False,
            "ping360_config_path": "",
            "ping360_overrides": None,
            "sitl_ip": "127.0.0.1",
            "sitl_port": 0,
            "sitl_send_port": 0,
            "sitl_mavlink_endpoint": "",
            "sitl_mavlink_servo_hz": 50.0,
            "sitl_mavlink_target_sysid": 1,
            "sitl_mavlink_target_compid": 1,
            "sitl_mavlink_source_sysid": 1,
            "sitl_mavlink_source_compid": 1,
        }

        with (
            patch(
                "bridge.ros2_bridge_constructor.ros2_bridge_runtime_setup."
                "configure_bridge_runtime_state",
                side_effect=configure_base,
            ),
            patch(
                "bridge.ros2_bridge_constructor.ros2_bridge_runtime_setup."
                "configure_command_runtime_state"
            ),
            patch(
                "bridge.ros2_bridge_constructor.ros2_bridge_runtime_setup."
                "configure_bridge_contracts",
                side_effect=configure_contracts,
            ),
            patch(
                "bridge.ros2_bridge_constructor.ros2_bridge_sensor_setup."
                "configure_mujoco_sensor_runtime"
            ),
            patch(
                "bridge.ros2_bridge_constructor.ros2_sitl_transport_setup."
                "create_sitl_transport_if_enabled",
                return_value=transport,
            ),
            patch(
                "bridge.ros2_bridge_constructor.ros2_bridge_init."
                "configure_ros_runtime_state",
                side_effect=configure_ros,
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "partial ROS"):
                initialize_ros2_bridge(bridge, init)

        emulator = resources["emulator"]
        self.assertFalse(emulator.is_running)
        self.assertIsNone(bridge._dvl_device_emulator)
        self.assertIsNone(bridge._sitl_poll_thread)
        self.assertTrue(transport.was_shutdown)
        self.assertIsNone(bridge._sitl_transport)
        self.assertIs(executor.removed, node)
        self.assertTrue(executor.was_shutdown)
        self.assertTrue(node.was_destroyed)
        self.assertTrue(context.was_shutdown)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rebound:
            rebound.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            rebound.bind(resources["address"])

    def test_model_disabled_preserves_legacy_ideal_odometry_integration(self) -> None:
        bridge = FakeLegacyBridge()
        with patch.dict(
            os.environ,
            {"ROS2_UUV_DVL_SENSOR_ENABLE": "false"},
            clear=False,
        ):
            configure_dvl_sensor_runtime(bridge)

        prepare_ros_publish_state(bridge, SimpleNamespace(time=0.0), sensor_snapshot())
        prepare_ros_publish_state(bridge, SimpleNamespace(time=0.1), sensor_snapshot())

        np.testing.assert_allclose(bridge._odom_pos, (0.2, 0.0, 0.0), atol=1.0e-12)

    def test_position_is_driven_by_its_own_delivery_cadence(self) -> None:
        latest = SimpleNamespace(
            capture_time_s=1.25,
            arrival_time_s=1.27,
            report_time_s=1.26,
            position_local_frd_m=(1.0, 2.0, 3.0),
            attitude_rpy_deg=(4.0, 5.0, 6.0),
            position_std_m=0.2,
        )
        bridge = SimpleNamespace(
            _dvl_sensor_model_enabled=True,
            DVLDRMsg=DvldrMsg,
        )
        state = SimpleNamespace(
            dvl_position_delivery=latest,
            quat_ros=np.array((1.0, 0.0, 0.0, 0.0), dtype=np.float64),
        )

        message = build_dvl_position_msg(bridge, Stamp(), state)

        self.assertEqual(
            (message.header.stamp.sec, message.header.stamp.nanosec),
            (1, 270_000_000),
        )
        self.assertEqual(
            (message.position.x, message.position.y, message.position.z),
            (1.0, 2.0, 3.0),
        )
        self.assertEqual((message.roll, message.pitch, message.yaw), (4.0, 5.0, 6.0))

        self.assertAlmostEqual(message.time, 1.26, places=12)

        state.dvl_position_delivery = None
        self.assertIsNone(build_dvl_position_msg(bridge, Stamp(), state))

    def test_raw_velocity_header_uses_arrival_and_time_uses_report_gap(self) -> None:
        sample = A50SensorModel(
            A50SensorConfig(
                white_noise_std_mps=0.0,
                velocity_noise_per_meter_mps=0.0,
                range_noise_std_m=0.0,
                range_noise_fraction=0.0,
            )
        ).sample(
            (0.2, 0.0, 0.0),
            2.0,
            time_of_validity_us=200_000,
            time_of_transmission_us=202_000,
        )
        delivery = SimpleNamespace(
            sample=sample,
            capture_time_s=0.2,
            arrival_time_s=0.23,
            report_period_s=0.2,
        )
        bridge = SimpleNamespace(
            _dvl_sensor_model_enabled=True,
            _ros_rate_dvl_twist_hz=10.0,
            DVLMsg=DvlMsg,
        )
        state = SimpleNamespace(dvl_sensor_delivery=delivery)

        message = build_dvl_data_msg(bridge, Stamp(), state)

        self.assertEqual(
            (message.header.stamp.sec, message.header.stamp.nanosec),
            (0, 230_000_000),
        )
        self.assertEqual(message.time, 200.0)
        self.assertEqual(message.time_of_validity, 200_000)
        self.assertEqual(message.time_of_transmission, 202_000)

    def test_invalid_direct_raw_covariance_matches_tcp_sentinel(self) -> None:
        sample = A50SensorModel(
            A50SensorConfig(
                white_noise_std_mps=0.0,
                velocity_noise_per_meter_mps=0.0,
            )
        ).sample(
            (0.2, 0.0, 0.0),
            None,
            time_of_validity_us=100_000,
            time_of_transmission_us=102_000,
        )
        delivery = SimpleNamespace(
            sample=sample,
            capture_time_s=0.1,
            arrival_time_s=0.12,
            report_period_s=0.1,
        )
        bridge = SimpleNamespace(
            _dvl_sensor_model_enabled=True,
            _ros_rate_dvl_twist_hz=10.0,
            DVLMsg=DvlMsg,
        )
        direct = build_dvl_data_msg(
            bridge,
            Stamp(),
            SimpleNamespace(dvl_sensor_delivery=delivery),
        )
        tcp = A50TcpJsonEmulator(port=0)._build_velocity_report(sample, 0.1)
        tcp_covariance = [value for row in tcp["covariance"] for value in row]

        self.assertFalse(direct.velocity_valid)
        self.assertEqual(direct.fom, tcp["fom"])
        self.assertEqual(direct.covariance, tcp_covariance)

    def test_sensor_state_advances_before_ros_disabled_guard(self) -> None:
        bridge = SimpleNamespace(_enable_ros=False, _ros_ok=False)
        prepared_state = object()
        data = SimpleNamespace(time=2.5)
        snapshot = object()

        with patch(
            "bridge.ros2_bridge_publish_ros.prepare_ros_publish_state",
            return_value=prepared_state,
        ) as prepare:
            publish_ros_snapshot(bridge, data, 2.5, snapshot)

        prepare.assert_called_once_with(bridge, data, snapshot)

    def test_ros_publish_failure_does_not_stop_real_device_tcp_stream(self) -> None:
        bridge = FakeLegacyBridge()
        configure_dvl_sensor_runtime(bridge)
        environment = {
            "ROS2_UUV_DVL_DEVICE_EMULATOR_ENABLE": "1",
            "ROS2_UUV_DVL_DEVICE_EMULATOR_HOST": "127.0.0.1",
            "ROS2_UUV_DVL_DEVICE_EMULATOR_PORT": "0",
        }
        with patch.dict(os.environ, environment, clear=False):
            configure_dvl_device_emulator_runtime(bridge)
        self.addCleanup(close_dvl_device_emulator, bridge)
        client = socket.create_connection(bridge._dvl_device_emulator.address)
        self.addCleanup(client.close)
        client.settimeout(1.0)
        deadline = time.monotonic() + 1.0
        while (
            not bridge._dvl_device_emulator.client_connected
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        self.assertTrue(bridge._dvl_device_emulator.client_connected)

        bridge._enable_ros = True
        bridge._ros_ok = True
        bridge._publish_static_context = lambda _stamp, _sim_t: True

        def fail_publish(*_args) -> None:
            raise RuntimeError("injected unrelated ROS failure")

        bridge._flush_ros_publish_jobs = fail_publish
        with patch(
            "bridge.ros2_bridge_publish_ros.acquire_ros_stamp",
            return_value=Stamp(),
        ):
            data = SimpleNamespace(time=0.0)
            with self.assertRaisesRegex(RuntimeError, "unrelated ROS"):
                publish_ros_snapshot(bridge, data, 0.0, sensor_snapshot())
            bridge._ros_ok = False
            data.time = 0.1
            publish_ros_snapshot(bridge, data, 0.1, sensor_snapshot())
            data.time = 0.13
            publish_ros_snapshot(bridge, data, 0.13, sensor_snapshot())

        stream = client.makefile("rb")
        self.addCleanup(stream.close)
        reports = [json.loads(stream.readline()) for _ in range(3)]
        velocity_reports = [report for report in reports if report["type"] == "velocity"]
        self.assertEqual(len(velocity_reports), 2)
        self.assertGreater(
            velocity_reports[1]["time_of_validity"],
            velocity_reports[0]["time_of_validity"],
        )
        self.assertGreaterEqual(bridge._dvl_sensor_timing.stats.delivered, 2)

    def test_device_streams_are_forwarded_in_cross_transport_arrival_order(self) -> None:
        velocity = SimpleNamespace(arrival_time_s=0.03)
        position = SimpleNamespace(arrival_time_s=0.01)
        bridge = SimpleNamespace(
            _dvl_sensor_new_deliveries=(velocity,),
            _dvl_sensor_new_position_deliveries=(position,),
        )
        forwarded = []

        with (
            patch(
                "bridge.ros2_publish_state.forward_dvl_delivery_to_device_emulator",
                side_effect=lambda _bridge, delivery: forwarded.append(
                    ("velocity", delivery.arrival_time_s)
                ),
            ),
            patch(
                "bridge.ros2_publish_state.forward_dvl_position_to_device_emulator",
                side_effect=lambda _bridge, delivery: forwarded.append(
                    ("position", delivery.arrival_time_s)
                ),
            ),
        ):
            _forward_dvl_device_deliveries(bridge)

        self.assertEqual(forwarded, [("position", 0.01), ("velocity", 0.03)])

    def test_direct_raw_ros_preserves_every_arrived_sensor_report(self) -> None:
        class RecordingJobs:
            def __init__(self) -> None:
                self.items = []

            def add(self, publisher, label, message) -> None:
                self.items.append((publisher, label, message))

        bridge = SimpleNamespace(
            _dvl_sensor_model_enabled=True,
            pub_dvl_data="data-publisher",
            pub_dvl_position="position-publisher",
        )
        jobs = RecordingJobs()
        builders = {
            "dvl_data_batch": lambda: ("data-0", "data-1", "data-2"),
            "dvl_position_batch": lambda: ("position-0", "position-1"),
        }

        schedule_real_dvl_compat_jobs(
            bridge,
            jobs,
            lambda *_args, **_kwargs: self.fail("rate path must not be used"),
            builders=builders,
        )

        self.assertEqual(
            jobs.items,
            [
                ("data-publisher", "/dvl/data", "data-0"),
                ("data-publisher", "/dvl/data", "data-1"),
                ("data-publisher", "/dvl/data", "data-2"),
                ("position-publisher", "/dvl/position", "position-0"),
                ("position-publisher", "/dvl/position", "position-1"),
            ],
        )

    def test_direct_raw_batch_builds_each_delivery_not_only_latest(self) -> None:
        @dataclass(frozen=True)
        class BatchState:
            dvl_sensor_delivery: object | None = None
            dvl_position_delivery: object | None = None

        sample_model = A50SensorModel(
            A50SensorConfig(
                white_noise_std_mps=0.0,
                velocity_noise_per_meter_mps=0.0,
                range_noise_std_m=0.0,
                range_noise_fraction=0.0,
            )
        )
        deliveries = []
        for index, arrival_time_s in enumerate((0.03, 0.13)):
            validity_us = index * 100_000
            deliveries.append(
                SimpleNamespace(
                    sample=sample_model.sample(
                        (0.1, 0.0, 0.0),
                        2.0,
                        time_of_validity_us=validity_us,
                        time_of_transmission_us=validity_us + 2_000,
                    ),
                    capture_time_s=index * 0.1,
                    arrival_time_s=arrival_time_s,
                    report_period_s=0.1,
                )
            )
        bridge = SimpleNamespace(
            _dvl_sensor_model_enabled=True,
            _dvl_sensor_new_deliveries=tuple(deliveries),
            _dvl_sensor_new_position_deliveries=(),
            _ros_rate_dvl_twist_hz=10.0,
            DVLMsg=DvlMsg,
            DVLDRMsg=DvldrMsg,
        )

        messages = DvlPublishBuilderCache(
            bridge,
            Stamp(),
            BatchState(),
        ).builders()["dvl_data_batch"]()

        self.assertEqual(len(messages), 2)
        self.assertEqual(
            [message.header.stamp.nanosec for message in messages],
            [30_000_000, 130_000_000],
        )

    def test_device_mode_suppresses_duplicate_direct_ros_publishers(self) -> None:
        class FailingNode:
            @staticmethod
            def create_publisher(*_args, **_kwargs):
                raise AssertionError("direct DVL publisher must not be constructed")

        bridge = SimpleNamespace(
            node=FailingNode(),
            DVLMsg=object,
            DVLDRMsg=object,
            _dvl_device_emulator_enabled=True,
        )
        create_dvl_compat_publishers(bridge, dvl_sensor_qos=object())
        self.assertIsNone(bridge.pub_dvl_data)
        self.assertIsNone(bridge.pub_dvl_position)

    def test_direct_raw_publishers_share_sensor_data_qos(self) -> None:
        class RecordingNode:
            def __init__(self) -> None:
                self.calls = []

            def create_publisher(self, message_type, topic, qos):
                self.calls.append((message_type, topic, qos))
                return topic

        sensor_qos = object()
        node = RecordingNode()
        bridge = SimpleNamespace(
            node=node,
            DVLMsg=DvlMsg,
            DVLDRMsg=DvldrMsg,
            _dvl_device_emulator_enabled=False,
        )

        create_dvl_compat_publishers(bridge, dvl_sensor_qos=sensor_qos)

        self.assertEqual(
            [(topic, qos) for _, topic, qos in node.calls],
            [("/dvl/data", sensor_qos), ("/dvl/position", sensor_qos)],
        )

    def test_dvl_sensor_qos_matches_physical_driver_profile(self) -> None:
        class QoSProfile:
            def __init__(self, **kwargs) -> None:
                self.__dict__.update(kwargs)

        bridge = SimpleNamespace(
            QoSProfile=QoSProfile,
            HistoryPolicy=SimpleNamespace(KEEP_LAST="keep_last"),
            ReliabilityPolicy=SimpleNamespace(
                BEST_EFFORT="best_effort",
                RELIABLE="reliable",
            ),
            DurabilityPolicy=SimpleNamespace(
                VOLATILE="volatile",
                TRANSIENT_LOCAL="transient_local",
            ),
        )

        with patch(
            "bridge.ros2_bridge_context_runtime.create_ros2_endpoints"
        ) as create_endpoints:
            initialize_ros2_endpoints(bridge)

        sensor_qos = create_endpoints.call_args.kwargs["dvl_sensor_qos"]
        self.assertEqual(sensor_qos.depth, 5)
        self.assertEqual(sensor_qos.history, "keep_last")
        self.assertEqual(sensor_qos.reliability, "best_effort")
        self.assertEqual(sensor_qos.durability, "volatile")


if __name__ == "__main__":
    unittest.main(verbosity=2)

#!/usr/bin/env python3
"""Regression tests for timed IMU/Bar30 ROS publication."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.bar30_sensor_model import Bar30SensorSample  # noqa: E402
from bridge.imu_sensor_model import ImuSensorSample  # noqa: E402
from bridge.ros2_imu_bar30_sensor_runtime import (  # noqa: E402
    Bar30SensorDelivery,
    ImuSensorDelivery,
)
from bridge.ros2_publish_core_cache import CorePublishBuilderCache  # noqa: E402
from bridge.ros2_publish_core_factories import build_core_depth_pose_msg  # noqa: E402
from bridge.ros2_publish_mavros_cache import MavrosPublishBuilderCache  # noqa: E402
from bridge.ros2_publish_mavros_cache_imu import (  # noqa: E402
    build_imu_raw_msg,
    build_mavros_imu_msg,
)
from bridge.ros2_publish_mavros_cache_status import build_static_pressure_msg  # noqa: E402
from bridge.ros2_endpoint_mavros_publishers import create_mavros_publishers  # noqa: E402
from bridge.ros2_publish_schedule_core import schedule_core_ros_jobs  # noqa: E402
from bridge.ros2_publish_schedule_mavros import (  # noqa: E402
    _schedule_mavros_sensor_jobs,
    schedule_mavros_ros_jobs,
)


class Stamp:
    def __init__(self) -> None:
        self.sec = 0
        self.nanosec = 0


class Header:
    def __init__(self) -> None:
        self.stamp = Stamp()
        self.frame_id = ""


class Vector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Quaternion:
    def __init__(self) -> None:
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class ImuMsg:
    def __init__(self) -> None:
        self.header = Header()
        self.orientation = Quaternion()
        self.angular_velocity = Vector3()
        self.linear_acceleration = Vector3()
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9


class FluidPressureMsg:
    def __init__(self) -> None:
        self.header = Header()
        self.fluid_pressure = 0.0
        self.variance = 0.0


class Float32Msg:
    def __init__(self) -> None:
        self.data = 0.0


class PoseCovMsg:
    def __init__(self) -> None:
        self.header = Header()
        position = Vector3()
        orientation = Quaternion()
        self.pose = SimpleNamespace(
            pose=SimpleNamespace(position=position, orientation=orientation),
            covariance=[0.0] * 36,
        )


class PressureLaw:
    @staticmethod
    def frontend_depth_m_from_pressure(pressure_pa: float) -> float:
        return (float(pressure_pa) - 100_000.0) / 10_000.0


def imu_delivery(capture_time_s: float, value: float) -> ImuSensorDelivery:
    sample = ImuSensorSample(
        sample_index=int(round(capture_time_s * 100.0)),
        sample_time_s=capture_time_s,
        orientation_wxyz=(0.5, 0.5, 0.5, 0.5),
        angular_velocity_rad_s=(value, 2.0, 3.0),
        linear_acceleration_mps2=(4.0, 5.0, 6.0),
        gyro_bias_rad_s=(0.0, 0.0, 0.0),
        accel_bias_mps2=(0.0, 0.0, 0.0),
        orientation_covariance_diag_rad2=(0.01, 0.02, 0.03),
        angular_velocity_covariance_diag_rad2_s2=(0.04, 0.05, 0.06),
        linear_acceleration_covariance_diag_m2_s4=(0.07, 0.08, 0.09),
        gyro_saturated=(False, False, False),
        accel_saturated=(False, False, False),
    )
    return ImuSensorDelivery(
        sample=sample,
        capture_time_s=capture_time_s,
        device_time_s=capture_time_s + 0.25,
        transmission_time_s=capture_time_s + 0.01,
        arrival_time_s=capture_time_s + 0.03,
    )


def bar_delivery(capture_time_s: float, pressure_pa: float) -> Bar30SensorDelivery:
    sample = Bar30SensorSample(
        sample_index=int(round(capture_time_s * 10.0)),
        sample_time_s=capture_time_s,
        true_pressure_pa=pressure_pa - 10.0,
        measured_pressure_pa=pressure_pa,
        sensor_temperature_c=20.0,
        total_bias_pa=10.0,
        variance_pa2=321.0,
        saturated=False,
    )
    return Bar30SensorDelivery(
        sample=sample,
        capture_time_s=capture_time_s,
        device_time_s=capture_time_s + 0.4,
        transmission_time_s=capture_time_s + 0.02,
        arrival_time_s=capture_time_s + 0.05,
    )


def fake_bridge():
    return SimpleNamespace(
        Imu=ImuMsg,
        FluidPressure=FluidPressureMsg,
        Float32=Float32Msg,
        PoseWithCovarianceStamped=PoseCovMsg,
        _imu_sensor_model_enabled=True,
        _bar30_sensor_model_enabled=True,
        _static_pressure_source="external",
        _baro_pressure_law=PressureLaw(),
        _ros_imu_accel_surface=lambda value: value,
    )


@dataclass
class FakeState:
    imu_sensor_delivery: object | None
    bar30_sensor_delivery: object | None
    imu_sensor_deliveries: tuple[object, ...]
    bar30_sensor_deliveries: tuple[object, ...]
    quat_ros: object = None
    gyro_ros: object = None
    acc_ros_surface: object = None
    ros_depth_m: float = 0.0
    bar30_pressure_pa: float = 100_000.0
    static_pressure_pa: float = 100_000.0


def fake_state(imu_deliveries=(), bar_deliveries=()):
    return FakeState(
        imu_sensor_delivery=imu_deliveries[-1] if imu_deliveries else None,
        bar30_sensor_delivery=bar_deliveries[-1] if bar_deliveries else None,
        imu_sensor_deliveries=tuple(imu_deliveries),
        bar30_sensor_deliveries=tuple(bar_deliveries),
        quat_ros=None,
        gyro_ros=None,
        acc_ros_surface=None,
        ros_depth_m=0.0,
        bar30_pressure_pa=100_000.0,
        static_pressure_pa=100_000.0,
    )


class PublishMessageTest(unittest.TestCase):
    def test_mavros_imu_uses_capture_stamp_frame_and_modeled_covariance(self) -> None:
        delivery = imu_delivery(1.25, 7.0)
        msg = build_mavros_imu_msg(fake_bridge(), Stamp(), fake_state((delivery,)))
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (1, 250_000_000))
        self.assertEqual(msg.header.frame_id, "fcu_link")
        self.assertEqual(
            (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z),
            (0.5, 0.5, 0.5, 0.5),
        )
        self.assertEqual(msg.angular_velocity.x, 7.0)
        self.assertEqual(
            [msg.orientation_covariance[index] for index in (0, 4, 8)],
            [0.01, 0.02, 0.03],
        )
        self.assertEqual(
            [msg.angular_velocity_covariance[index] for index in (0, 4, 8)],
            [0.04, 0.05, 0.06],
        )
        self.assertNotEqual(delivery.capture_time_s, delivery.arrival_time_s)

    def test_strict_mavros_raw_imu_uses_physical_bag_orientation_contract(self) -> None:
        delivery = imu_delivery(1.25, 7.0)
        bridge = fake_bridge()
        bridge._strict_sitl_sensor_transport = True
        msg = build_imu_raw_msg(bridge, Stamp(), fake_state((delivery,)))

        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (1, 250_000_000))
        self.assertEqual(msg.header.frame_id, "fcu_link")
        self.assertEqual(
            (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z),
            (1.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(msg.orientation_covariance, [-1.0] + [0.0] * 8)
        self.assertEqual(
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            (7.0, 2.0, 3.0),
        )
        self.assertEqual(
            (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ),
            (4.0, 5.0, 6.0),
        )
        self.assertEqual(
            [msg.angular_velocity_covariance[index] for index in (0, 4, 8)],
            [0.04, 0.05, 0.06],
        )
        self.assertEqual(
            [msg.linear_acceleration_covariance[index] for index in (0, 4, 8)],
            [0.07, 0.08, 0.09],
        )

    def test_non_strict_mavros_raw_imu_preserves_legacy_orientation(self) -> None:
        delivery = imu_delivery(1.25, 7.0)
        bridge = fake_bridge()
        bridge._strict_sitl_sensor_transport = False
        msg = build_imu_raw_msg(bridge, Stamp(), fake_state((delivery,)))

        self.assertEqual(
            (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z),
            (0.5, 0.5, 0.5, 0.5),
        )
        self.assertEqual(
            [msg.orientation_covariance[index] for index in (0, 4, 8)],
            [0.01, 0.02, 0.03],
        )

    def test_static_pressure_preserves_real_topic_frame_and_variance(self) -> None:
        delivery = bar_delivery(2.5, 123_456.0)
        msg = build_static_pressure_msg(fake_bridge(), Stamp(), fake_state(bar_deliveries=(delivery,)))
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (2, 500_000_000))
        self.assertEqual(msg.header.frame_id, "fcu_link")
        self.assertEqual(msg.fluid_pressure, 123_456.0)
        self.assertEqual(msg.variance, 321.0)

    def test_depth_pose_is_derived_only_from_delivered_pressure(self) -> None:
        delivery = bar_delivery(3.0, 121_000.0)
        state = fake_state(bar_deliveries=(delivery,))
        state.ros_depth_m = 99.0
        msg = build_core_depth_pose_msg(fake_bridge(), Stamp(), state)
        self.assertEqual(msg.header.frame_id, "odom")
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (3, 0))
        self.assertEqual(msg.pose.pose.position.z, -2.1)

    def test_no_arrival_means_no_modeled_ros_message(self) -> None:
        bridge = fake_bridge()
        state = fake_state()
        self.assertIsNone(build_mavros_imu_msg(bridge, Stamp(), state))
        self.assertIsNone(build_imu_raw_msg(bridge, Stamp(), state))
        self.assertIsNone(build_static_pressure_msg(bridge, Stamp(), state))


class PublishBatchTest(unittest.TestCase):
    def test_mavros_cache_retains_every_arrived_capture(self) -> None:
        deliveries = (imu_delivery(1.0, 1.0), imu_delivery(1.02, 2.0))
        cache = MavrosPublishBuilderCache(fake_bridge(), Stamp(), fake_state(deliveries))
        messages = cache.builders()["mavros_imu_batch"]()
        self.assertEqual(len(messages), 2)
        self.assertEqual([message.angular_velocity.x for message in messages], [1.0, 2.0])
        self.assertEqual(
            [(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages],
            [(1, 0), (1, 20_000_000)],
        )

    def test_core_cache_retains_every_pressure_capture(self) -> None:
        deliveries = (bar_delivery(1.0, 110_000.0), bar_delivery(1.1, 120_000.0))
        cache = CorePublishBuilderCache(
            fake_bridge(), Stamp(), fake_state(bar_deliveries=deliveries)
        )
        messages = cache.builders()["baro_batch"]()
        self.assertEqual([message.data for message in messages], [110_000.0, 120_000.0])

    def test_mavros_schedule_is_delivery_driven_not_second_rate_gate(self) -> None:
        bridge = fake_bridge()
        bridge.pub_mavros_imu_data = "imu"
        bridge.pub_mavros_imu_data_raw = "raw"
        bridge.pub_mavros_imu_static_pressure = "pressure"
        bridge.pub_mavros_imu_atm_pressure = None
        bridge.pub_mavros_battery = None
        bridge._ros_rate_mavros_imu_data_hz = 1.0
        bridge._ros_rate_mavros_imu_raw_hz = 1.0
        bridge._ros_rate_mavros_static_pressure_hz = 1.0
        bridge._ros_rate_mavros_atm_pressure_hz = 1.0
        jobs = SimpleNamespace(items=[])
        jobs.add = lambda publisher, label, message, **_kwargs: (
            jobs.items.append((publisher, label, message))
            if publisher is not None
            else None
        )
        rate_limited = []
        builders = {
            "mavros_imu_batch": lambda: ("i0", "i1"),
            "mavros_imu_raw_batch": lambda: ("r0", "r1"),
            "mavros_static_pressure_batch": lambda: ("p0",),
            "mavros_atm_pressure": object(),
            "mavros_battery": object(),
        }
        _schedule_mavros_sensor_jobs(
            bridge,
            jobs,
            lambda *args: rate_limited.append(args),
            builders=builders,
        )
        self.assertEqual(
            [(publisher, label) for publisher, label, _ in jobs.items],
            [
                ("imu", "/mavros/imu/data"),
                ("imu", "/mavros/imu/data"),
                ("raw", "/mavros/imu/data_raw"),
                ("raw", "/mavros/imu/data_raw"),
                ("pressure", "/mavros/imu/static_pressure"),
            ],
        )
        self.assertEqual(len(rate_limited), 1)
        self.assertEqual(rate_limited[0][1], "/mavros/imu/atm_pressure")

    def test_strict_sitl_surface_owns_only_delivery_driven_mavros_sensors(self) -> None:
        class Node:
            def __init__(self) -> None:
                self.publishers = {}

            def create_publisher(self, _msg_type, topic, qos):
                self.publishers[topic] = qos
                return topic

        node = Node()
        bridge = SimpleNamespace(
            node=node,
            _mavros_surface_enabled=False,
            _strict_sitl_sensor_transport=True,
            RCOut=None,
        )
        for attr in (
            "VfrHud",
            "MavrosState",
            "Imu",
            "FluidPressure",
            "PoseStamped",
            "Odometry",
            "TwistStamped",
            "TwistWithCovarianceStamped",
            "BatteryState",
            "RCIn",
        ):
            setattr(bridge, attr, object)

        regular_qos = object()
        sensor_qos = object()
        create_mavros_publishers(
            bridge,
            q10=regular_qos,
            latched_qos=object(),
            sensor_qos=sensor_qos,
        )

        self.assertEqual(
            set(node.publishers),
            {
                "/mavros/imu/data_raw",
                "/mavros/imu/static_pressure",
            },
        )
        self.assertTrue(all(qos is sensor_qos for qos in node.publishers.values()))
        self.assertIsNone(bridge.pub_mavros_imu_data)
        self.assertIsNone(bridge.pub_mavros_state)
        self.assertIsNone(bridge.pub_mavros_vfr_hud)

    def test_strict_sitl_schedule_does_not_repeat_latest_delivery(self) -> None:
        bridge = fake_bridge()
        bridge._mavros_surface_enabled = False
        bridge._strict_sitl_sensor_transport = True
        bridge.pub_mavros_imu_data = None
        bridge.pub_mavros_imu_data_raw = "raw"
        bridge.pub_mavros_imu_static_pressure = "pressure"
        for attr in (
            "pub_mavros_state",
            "pub_mavros_vfr_hud",
            "pub_mavros_imu_atm_pressure",
            "pub_mavros_battery",
            "pub_mavros_local_pose",
            "pub_mavros_local_vel",
            "pub_mavros_local_vel_body",
            "pub_mavros_local_vel_body_cov",
            "pub_mavros_vision_pose",
            "pub_mavros_rc_in",
            "pub_mavros_rc_out",
        ):
            setattr(bridge, attr, None)
        bridge._mavros_last_rc_override = None
        bridge._mavros_last_rc_out = None
        bridge._mavros_rc_out_publish_mode = "event"
        bridge._mavros_state_pub_hz = 1.0
        bridge._mavros_state_next_t = 0.0

        jobs = SimpleNamespace(items=[])
        jobs.add = lambda publisher, label, message, **_kwargs: (
            jobs.items.append((publisher, label, message))
            if publisher is not None
            else None
        )
        builders = {
            "mavros_imu_batch": lambda: self.fail(
                "strict bridge must not build the external MAVROS AHRS topic"
            ),
            "mavros_imu_raw_batch": lambda: ("r0",),
            "mavros_static_pressure_batch": lambda: ("p0",),
        }
        schedule_mavros_ros_jobs(
            bridge,
            jobs,
            lambda *_args, **_kwargs: self.fail("strict sensor path must not rate-repeat"),
            1.0,
            builders=builders,
        )
        self.assertEqual(
            [(publisher, label) for publisher, label, _ in jobs.items],
            [
                ("raw", "/mavros/imu/data_raw"),
                ("pressure", "/mavros/imu/static_pressure"),
            ],
        )

        jobs.items.clear()
        builders.update(
            {
                "mavros_imu_batch": lambda: self.fail(
                    "empty strict cycle must not build the external AHRS topic"
                ),
                "mavros_imu_raw_batch": lambda: (),
                "mavros_static_pressure_batch": lambda: (),
            }
        )
        schedule_mavros_ros_jobs(
            bridge,
            jobs,
            lambda *_args, **_kwargs: self.fail("empty delivery must stay empty"),
            1.01,
            builders=builders,
        )
        self.assertEqual(jobs.items, [])

    def test_core_schedule_is_delivery_driven(self) -> None:
        bridge = fake_bridge()
        bridge._real_pkg_compat = False
        bridge.pub_imu = "imu"
        bridge.pub_depth = "depth"
        bridge.pub_depth_pose = "pose"
        bridge.pub_bar30_pressure = "baro"
        for attr in (
            "pub_battery",
            "pub_mujoco_sim_time",
            "pub_sitl_sensor_replay_status",
            "pub_sitl_mavlink_telemetry_status",
            "pub_ground_truth",
            "pub_course_buoy_status",
            "pub_collector_state",
        ):
            setattr(bridge, attr, None)
        jobs = SimpleNamespace(items=[])
        jobs.add = lambda publisher, label, message, **_kwargs: (
            jobs.items.append((publisher, label, message))
            if publisher is not None
            else None
        )
        builders = {
            "imu_batch": lambda: ("i0", "i1"),
            "depth_batch": lambda: ("d0",),
            "depth_pose_batch": lambda: ("z0",),
            "baro_batch": lambda: ("b0",),
            "mavros_battery": object(),
            "sim_time": object(),
            "sitl_sensor_replay_status": object(),
            "sitl_mavlink_telemetry_status": object(),
            "ground_truth": object(),
            "course_buoy_status": object(),
            "collector_state": object(),
        }
        schedule_core_ros_jobs(
            bridge,
            jobs,
            lambda *_args, **_kwargs: None,
            builders=builders,
        )
        self.assertEqual(
            [(publisher, label) for publisher, label, _ in jobs.items],
            [
                ("imu", "/imu/data"),
                ("imu", "/imu/data"),
                ("depth", "/depth"),
                ("pose", "/depth/pose"),
                ("baro", "/bar30/pressure_pa"),
            ],
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)

"""Regression tests for sensor-only external navigation and packet delivery."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge.ros2_publish_mavros_cache_local import build_vision_pose_msg  # noqa: E402
from bridge.ros2_vision_sensor_pose import VisionSensorPoseRuntime  # noqa: E402


@dataclass
class Stamp:
    sec: int = 0
    nanosec: int = 0


class PoseMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=Stamp(), frame_id="")
        self.pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0., z=0.),
                                    orientation=SimpleNamespace(w=1., x=0., y=0., z=0.))


@dataclass
class State:
    sim_t: float = 1.3
    dvl_position_delivery: object = None
    dvl_position_deliveries: tuple = ()
    imu_sensor_deliveries: tuple = ()
    bar30_sensor_deliveries: tuple = ()

    @property
    def base_pos_enu(self):
        raise AssertionError("external navigation read live GT position")

    @property
    def quat_ros(self):
        raise AssertionError("external navigation read live GT orientation")


def report(**overrides):
    fields = dict(capture_time_s=1., arrival_time_s=1.05, report_time_s=20.,
                  position_local_frd_m=(1., 2., 3.), position_std_m=.02,
                  attitude_rpy_deg=(0., 0., 90.), generation=0, navigation_valid=True)
    fields.update(overrides)
    return SimpleNamespace(**fields)


def owner(*, lever=(0., 0., 0.), initial=(0., 0., 0.)):
    runtime = VisionSensorPoseRuntime(PoseMessage, body_flu_to_dvl_frd=np.diag([1., -1., -1.]),
             dvl_position_body_flu_m=np.asarray(lever),
             config=VisionSensorPoseRuntime.Config(position_enu_m=initial))
    return SimpleNamespace(PoseStamped=PoseMessage, _dvl_sensor_enabled=True,
                           _dvl_sensor_dr_generation=0, _vision_sensor_pose_runtime=runtime)


def position(message):
    return np.asarray([message.pose.position.x, message.pose.position.y, message.pose.position.z])


def test_public_vision_uses_sensor_pose_and_capture_stamp_without_truth():
    message = build_vision_pose_msg(owner(), Stamp(99), State(dvl_position_delivery=report()))
    np.testing.assert_allclose(position(message), [1., -2., -3.])
    np.testing.assert_allclose([message.pose.orientation.w, message.pose.orientation.z],
                               [np.sqrt(.5), -np.sqrt(.5)])
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (1, 0)


@pytest.mark.parametrize("changes", [
    {"navigation_valid": False}, {"generation": 1}, {"capture_time_s": .1},
    {"arrival_time_s": 2.}, {"capture_time_s": float("nan")},
    {"position_local_frd_m": (float("nan"), 0., 0.)}, {"position_std_m": -1.},
    {"capture_time_s": None}, {"position_local_frd_m": ("invalid", 0., 0.)},
])
def test_bad_sensor_report_never_falls_back_to_truth(changes):
    assert build_vision_pose_msg(owner(), Stamp(), State(dvl_position_delivery=report(**changes))) is None


def test_no_report_or_disabled_sensor_never_falls_back_to_truth():
    assert build_vision_pose_msg(owner(), Stamp(), State()) is None
    bridge = owner()
    bridge._dvl_sensor_enabled = False
    assert build_vision_pose_msg(bridge, Stamp(), State(dvl_position_delivery=report())) is None


def test_duplicate_or_out_of_order_capture_is_not_restamped():
    bridge = owner()
    first = State(dvl_position_delivery=report())
    assert build_vision_pose_msg(bridge, Stamp(), first) is not None
    assert build_vision_pose_msg(bridge, Stamp(), first) is None
    assert build_vision_pose_msg(bridge, Stamp(), State(dvl_position_delivery=report(capture_time_s=.9))) is None


def test_device_reset_continues_estimate_and_rejects_old_generation():
    bridge = owner()
    first = build_vision_pose_msg(bridge, Stamp(), State(dvl_position_delivery=report()))
    bridge._dvl_sensor_dr_generation = 1
    assert build_vision_pose_msg(bridge, Stamp(), State(dvl_position_delivery=report())) is None
    second = build_vision_pose_msg(bridge, Stamp(), State(sim_t=1.5, dvl_position_delivery=report(
        capture_time_s=1.4, arrival_time_s=1.45, generation=1,
        position_local_frd_m=(0., 0., 0.), attitude_rpy_deg=(0., 0., 0.))))
    np.testing.assert_allclose(position(second), position(first))
    assert second.pose.orientation.z == pytest.approx(first.pose.orientation.z)


def test_clock_rewind_restores_explicit_initial_pose():
    bridge = owner(initial=(4., 5., -2.))
    build_vision_pose_msg(bridge, Stamp(), State(dvl_position_delivery=report()))
    reset = build_vision_pose_msg(bridge, Stamp(), State(sim_t=.2, dvl_position_delivery=report(
        capture_time_s=.1, arrival_time_s=.15,
        position_local_frd_m=(0., 0., 0.), attitude_rpy_deg=(0., 0., 0.))))
    np.testing.assert_allclose(position(reset), [4., 5., -2.])


def test_rotating_sensor_lever_does_not_translate_base_estimate():
    # In local FRD a +90 degree yaw moves the x=1 sensor to y=1.
    bridge = owner(lever=(1., 0., 0.))
    message = build_vision_pose_msg(bridge, Stamp(), State(dvl_position_delivery=report(
        position_local_frd_m=(-1., 1., 0.))))
    np.testing.assert_allclose(position(message), [0., 0., 0.], atol=1e-12)


def test_lazy_cache_preserves_all_sensor_packets_once():
    from bridge.ros2_publish_mavros_cache import MavrosPublishBuilderCache
    bridge = owner()
    reports = (report(), report(capture_time_s=1.2, arrival_time_s=1.25))
    state = State(dvl_position_delivery=reports[-1], dvl_position_deliveries=reports)
    builders = MavrosPublishBuilderCache(bridge, Stamp(), state).builders()
    messages = builders["mavros_vision_pose_batch"]()
    assert len(messages) == 2
    assert [m.header.stamp.nanosec for m in messages] == [0, 200000000]
    assert builders["mavros_vision_pose_batch"]() is messages


def test_vision_arrival_flushes_between_general_ros_ticks():
    from bridge.ros2_publish_runtime import flush_sensor_packet_jobs
    from bridge.ros2_publisher_demand import PublisherDemandCache
    bridge = owner()
    bridge._real_pkg_compat = True
    bridge._mavros_surface_enabled = True
    bridge._imu_sensor_model_enabled = bridge._bar30_sensor_model_enabled = False
    bridge._publisher_demand = PublisherDemandCache()
    bridge.pub_mavros_vision_pose = "vision"
    delivered = []
    bridge._safe_publish = lambda publisher, message, label: delivered.append(message) or True
    state = State(dvl_position_delivery=report(), dvl_position_deliveries=(report(),))
    assert flush_sensor_packet_jobs(bridge, Stamp(), state)
    assert len(delivered) == 1
    assert delivered[0].header.stamp.sec == 1


def test_actual_dvl_runtime_tracks_readiness_dropout_and_reset():
    from test_dvl_sensor_runtime import FakeBridge, FakeState, sensor_config
    from bridge.ros2_dvl_sensor_runtime import (advance_dvl_sensor_runtime,
        configure_dvl_sensor_runtime, reset_dvl_dead_reckoning)
    with sensor_config():
        bridge = FakeBridge()
        configure_dvl_sensor_runtime(bridge)
        advance_dvl_sensor_runtime(bridge, FakeState(0., np.ones(3), 1., np.eye(3), np.zeros(3)))
        for i in range(1, 11):
            advance_dvl_sensor_runtime(bridge, FakeState(i * .02, np.ones(3), 1., np.eye(3), np.zeros(3)))
        assert bridge._dvl_sensor_last_position_delivery.navigation_valid
        for i in range(11, 21):
            advance_dvl_sensor_runtime(bridge, FakeState(i * .02, None, None, np.eye(3), np.zeros(3)))
        assert not bridge._dvl_sensor_last_position_delivery.navigation_valid
        reset_dvl_dead_reckoning(bridge)
        assert not bridge._dvl_sensor_navigation_valid


def test_static_nested_mount_does_not_read_dynamic_pose():
    bridge = owner()
    del bridge._vision_sensor_pose_runtime
    bridge._dvl_site_id, bridge._base_id = 0, 1
    bridge._dvl_body_frd_to_dvl_frd = np.eye(3)
    bridge._bmj_to_frd = np.diag([1., -1., -1.])
    bridge.model = SimpleNamespace(site_pos=np.asarray([[.1, 0., 0.]]), site_bodyid=np.asarray([2]),
        body_parentid=np.asarray([0, 0, 1]), body_jntnum=np.zeros(3),
        body_pos=np.asarray([[0., 0., 0.], [0., 0., 0.], [.2, 0., 0.]]),
        body_quat=np.asarray([[1., 0., 0., 0.]] * 3))
    runtime = VisionSensorPoseRuntime.from_bridge(bridge)
    np.testing.assert_allclose(runtime.lever_dvl, [.3, 0., 0.])


@pytest.mark.parametrize("sitl,enabled", [(True, True), (False, False)])
def test_native_external_nav_opt_in_does_not_spoof_fcu_topics(monkeypatch, sitl, enabled):
    from bridge.ros2_endpoint_mavros_publishers import create_mavros_publishers
    monkeypatch.setenv("ROS2_UUV_SENSOR_EXTERNAL_NAV", "1")
    topics = []
    def create(message_type, topic, qos):
        topics.append(topic)
        return topic
    bridge = SimpleNamespace(node=SimpleNamespace(create_publisher=create),
        _mavros_surface_enabled=False, _strict_sitl_sensor_transport=False,
        enable_sitl=sitl, PoseStamped=PoseMessage)
    create_mavros_publishers(bridge, q10=object(), latched_qos=object())
    assert topics == (["/mavros/vision_pose/pose"] if enabled else [])
    assert bridge.pub_mavros_state is None and bridge.pub_mavros_imu_data is None


def test_native_external_nav_schedule_uses_delivery_batch_only():
    from bridge.ros2_publish_schedule_mavros import schedule_mavros_ros_jobs
    bridge = SimpleNamespace(_mavros_surface_enabled=False, _sensor_external_nav_enabled=True,
                             _strict_sitl_sensor_transport=False, pub_mavros_vision_pose="vision")
    delivered, rate_jobs = [], []
    jobs = SimpleNamespace(add=lambda publisher, label, message: delivered.append(message))
    messages = (object(), object())
    schedule_mavros_ros_jobs(bridge, jobs, lambda *args, **kwargs: rate_jobs.append(args), 1.,
                            builders={"mavros_vision_pose_batch": lambda: messages})
    assert tuple(delivered) == messages and not rate_jobs


def test_scene_reset_reinitializes_navigation_at_monotonic_clock():
    from test_demo_reset import fixture
    from unittest.mock import Mock
    reset = fixture()
    reset.bridge.reset_odometry = Mock()
    reset.step.data.time = 123.
    reset.restore()
    reset.bridge.reset_odometry.assert_called_once_with(scene_reset=True)
    assert reset.step.data.time == 123.


def test_explicit_navigation_reset_restores_initial_pose_without_clock_rewind():
    from unittest.mock import patch
    from bridge.ros2_bridge_servo_api import reset_odometry
    bridge = owner(initial=(4., 5., -2.))
    build_vision_pose_msg(bridge, Stamp(), State(dvl_position_delivery=report()))
    with patch("bridge.ros2_bridge_servo_api.reset_dvl_dead_reckoning",
               side_effect=lambda target: setattr(target, "_dvl_sensor_dr_generation", 1)), \
         patch("bridge.ros2_bridge_servo_api.reset_dvl_device_emulator_dead_reckoning"), \
         patch("bridge.ros2_bridge_servo_api.reset_dvl_device_emulator_report_schedule"):
        reset_odometry(bridge)
    result = build_vision_pose_msg(bridge, Stamp(), State(sim_t=2., dvl_position_delivery=report(
        capture_time_s=1.9, arrival_time_s=1.95, position_local_frd_m=(0., 0., 0.),
        attitude_rpy_deg=(0., 0., 0.), generation=1)))
    np.testing.assert_allclose(position(result), [4., 5., -2.])

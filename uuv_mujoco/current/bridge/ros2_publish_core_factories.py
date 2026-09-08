"""Core ROS message factory functions for publish jobs."""

from __future__ import annotations

from collections.abc import Callable

from .ros2_standard_messages import build_depth_pose_msg, build_imu_msg, build_pose_msg
from .ros2_bridge_publish_stamp import stamp_from_seconds_like
from .ros2_imu_bar30_messages import build_delivered_imu_msg, delivered_depth_m

CoreFactory = Callable[[object, object, object], object]


def build_core_imu_msg(bridge, stamp, state):
    if bool(getattr(bridge, "_imu_sensor_model_enabled", False)):
        if state.imu_sensor_delivery is None:
            return None
        return build_delivered_imu_msg(
            bridge,
            stamp,
            state.imu_sensor_delivery,
            frame_id="imu_link",
        )
    return build_imu_msg(bridge.Imu, stamp, state.quat_ros, state.gyro_ros, state.acc_ros_surface)


def build_core_imu_raw_msg(bridge, stamp, state):
    if bool(getattr(bridge, "_imu_sensor_model_enabled", False)):
        if state.imu_sensor_delivery is None:
            return None
        return build_delivered_imu_msg(
            bridge,
            stamp,
            state.imu_sensor_delivery,
            frame_id="imu_link",
        )
    return build_imu_msg(bridge.Imu, stamp, state.quat_ros, state.gyro_ros, state.acc_ros_surface)


def build_core_depth_msg(bridge, _stamp, state):
    if bool(getattr(bridge, "_bar30_sensor_model_enabled", False)):
        if state.bar30_sensor_delivery is None:
            return None
        depth_m = delivered_depth_m(bridge, state.bar30_sensor_delivery)
    else:
        depth_m = state.ros_depth_m
    msg = bridge.Float32()
    msg.data = float(depth_m)
    return msg


def build_core_depth_pose_msg(bridge, stamp, state):
    if bool(getattr(bridge, "_bar30_sensor_model_enabled", False)):
        delivery = state.bar30_sensor_delivery
        if delivery is None:
            return None
        stamp = stamp_from_seconds_like(stamp, delivery.capture_time_s)
        depth_m = delivered_depth_m(bridge, delivery)
    else:
        depth_m = state.ros_depth_m
    return build_depth_pose_msg(bridge.PoseWithCovarianceStamped, stamp, depth_m)


def build_core_baro_msg(bridge, _stamp, state):
    if bool(getattr(bridge, "_bar30_sensor_model_enabled", False)):
        if state.bar30_sensor_delivery is None:
            return None
        pressure_pa = state.bar30_sensor_delivery.sample.measured_pressure_pa
    else:
        pressure_pa = state.bar30_pressure_pa
    msg = bridge.Float32()
    msg.data = float(pressure_pa)
    return msg


def build_core_ground_truth_msg(bridge, stamp, state):
    return build_pose_msg(bridge.PoseStamped, stamp, "world", state.base_pos_enu, state.quat_ros)


def build_core_sim_time_msg(bridge, _stamp, state):
    msg = bridge.Float32()
    msg.data = float(state.sim_t)
    return msg


CORE_PUBLISH_FACTORIES: tuple[tuple[str, CoreFactory], ...] = (
    ("imu", build_core_imu_msg),
    ("imu_raw", build_core_imu_raw_msg),
    ("depth", build_core_depth_msg),
    ("depth_pose", build_core_depth_pose_msg),
    ("baro", build_core_baro_msg),
    ("ground_truth", build_core_ground_truth_msg),
    ("sim_time", build_core_sim_time_msg),
)


__all__ = [
    "CORE_PUBLISH_FACTORIES",
    "CoreFactory",
    "build_core_baro_msg",
    "build_core_depth_msg",
    "build_core_depth_pose_msg",
    "build_core_ground_truth_msg",
    "build_core_imu_msg",
    "build_core_imu_raw_msg",
    "build_core_sim_time_msg",
]

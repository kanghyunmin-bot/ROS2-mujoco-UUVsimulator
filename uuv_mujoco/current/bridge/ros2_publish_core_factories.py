"""Core ROS message factory functions for publish jobs."""

from __future__ import annotations

from collections.abc import Callable

from .ros2_standard_messages import build_depth_pose_msg, build_imu_msg, build_pose_msg

CoreFactory = Callable[[object, object, object], object]


def build_core_imu_msg(bridge, stamp, state):
    return build_imu_msg(bridge.Imu, stamp, state.quat_ros, state.gyro_ros, state.acc_ros_surface)


def build_core_imu_raw_msg(bridge, stamp, state):
    return build_imu_msg(bridge.Imu, stamp, state.quat_ros, state.gyro_ros, state.acc_ros_surface)


def build_core_depth_msg(bridge, _stamp, state):
    msg = bridge.Float32()
    msg.data = float(state.ros_depth_m)
    return msg


def build_core_depth_pose_msg(bridge, stamp, state):
    return build_depth_pose_msg(bridge.PoseWithCovarianceStamped, stamp, state.ros_depth_m)


def build_core_baro_msg(bridge, _stamp, state):
    msg = bridge.Float32()
    msg.data = float(state.bar30_pressure_pa)
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

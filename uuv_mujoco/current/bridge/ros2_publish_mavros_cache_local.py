"""MAVROS-compatible local pose, velocity, odometry, and vision builders."""

from __future__ import annotations

from .ros2_standard_messages import build_odom_msg, build_pose_msg, build_twist_cov_msg, build_twist_msg


def build_local_pose_msg(bridge, stamp, state):
    return build_pose_msg(
        bridge.PoseStamped,
        stamp,
        "map",
        state.base_pos_enu,
        state.quat_ros,
    )


def build_local_vel_msg(bridge, stamp, state):
    return build_twist_msg(bridge.TwistStamped, stamp, "map", state.base_vel_enu)


def build_local_vel_body_msg(bridge, stamp, state):
    return build_twist_msg(
        bridge.TwistStamped,
        stamp,
        "base_link",
        state.base_vel_body_ros,
    )


def build_local_vel_body_cov_msg(bridge, stamp, state):
    return build_twist_cov_msg(
        bridge.TwistWithCovarianceStamped,
        stamp,
        "base_link",
        state.base_vel_body_ros,
        angular=state.gyro_ros,
    )


def build_local_odom_msg(bridge, stamp, state):
    return build_odom_msg(
        bridge.Odometry,
        stamp,
        "map",
        "base_link",
        state.base_pos_enu,
        state.quat_ros,
        state.base_vel_enu,
        angular_vel=state.gyro_ros,
    )


def build_vision_pose_msg(bridge, stamp, state):
    return build_pose_msg(
        bridge.PoseStamped,
        stamp,
        "map",
        state.base_pos_enu,
        state.quat_ros,
    )


__all__ = [
    "build_local_odom_msg",
    "build_local_pose_msg",
    "build_local_vel_body_cov_msg",
    "build_local_vel_body_msg",
    "build_local_vel_msg",
    "build_vision_pose_msg",
]

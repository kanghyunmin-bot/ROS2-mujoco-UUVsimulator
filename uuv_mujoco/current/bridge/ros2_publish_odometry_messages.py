"""Odometry message construction helpers for ROS publish jobs."""

from __future__ import annotations

import numpy as np

from .ros2_standard_messages import build_odom_msg


def odom_twist_velocity(state, zero_vel: np.ndarray) -> np.ndarray:
    return state.dvl_vel_body_ros if state.dvl_vel_body_ros is not None else zero_vel


def build_local_odom_msg(bridge, stamp, state, zero_vel: np.ndarray):
    return build_odom_msg(
        bridge.Odometry,
        stamp,
        "odom",
        "base_link",
        bridge._odom_pos,
        state.quat_ros,
        odom_twist_velocity(state, zero_vel),
    )


def build_rovio_odom_msg(bridge, stamp, state, quat_rovio: np.ndarray, zero_vel: np.ndarray):
    return build_odom_msg(
        bridge.Odometry,
        stamp,
        "odom",
        "base_link",
        bridge._odom_pos,
        quat_rovio,
        odom_twist_velocity(state, zero_vel),
    )


def build_sim_odom_msg(bridge, stamp, state):
    # MuJoCo base_link body position is the simulated robot center.
    # Pose is expressed in map/world ENU, while twist remains in
    # base_link FLU like nav_msgs/Odometry expects.
    return build_odom_msg(
        bridge.Odometry,
        stamp,
        "map",
        "base_link",
        state.base_pos_enu,
        state.quat_ros,
        state.base_vel_body_ros,
        angular_vel=state.gyro_ros,
    )


__all__ = [
    "build_local_odom_msg",
    "build_rovio_odom_msg",
    "build_sim_odom_msg",
    "odom_twist_velocity",
]

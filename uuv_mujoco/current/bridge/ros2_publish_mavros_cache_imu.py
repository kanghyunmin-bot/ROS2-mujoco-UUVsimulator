"""MAVROS-compatible IMU message builders."""

from __future__ import annotations

from .ros2_standard_messages import apply_real_mavros_imu_covariance, build_imu_msg


def build_mavros_imu_msg(bridge, stamp, state):
    # The April 1 real bag uses frame_id=fcu_link, but MAVROS has already
    # converted IMU vectors into ROS FLU convention.
    msg = build_imu_msg(
        bridge.Imu,
        stamp,
        state.quat_ros,
        state.gyro_ros,
        state.acc_ros_surface,
        frame_id="fcu_link",
    )
    apply_real_mavros_imu_covariance(msg)
    return msg


def build_imu_raw_msg(bridge, stamp, state):
    msg = build_imu_msg(
        bridge.Imu,
        stamp,
        state.quat_ros,
        state.gyro_ros,
        state.acc_ros_surface,
        frame_id="fcu_link",
    )
    apply_real_mavros_imu_covariance(msg)
    return msg


__all__ = ["build_imu_raw_msg", "build_mavros_imu_msg"]

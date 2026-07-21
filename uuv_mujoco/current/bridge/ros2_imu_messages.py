"""IMU ROS2 message builders and covariance contracts."""

from __future__ import annotations

from typing import Any

import numpy as np


def build_imu_msg(
    imu_type: type,
    stamp: Any,
    quat: np.ndarray,
    gyro: np.ndarray,
    acc: np.ndarray,
    *,
    frame_id: str = "imu_link",
) -> Any:
    msg = imu_type()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.orientation.w = float(quat[0])
    msg.orientation.x = float(quat[1])
    msg.orientation.y = float(quat[2])
    msg.orientation.z = float(quat[3])
    msg.angular_velocity.x = float(gyro[0])
    msg.angular_velocity.y = float(gyro[1])
    msg.angular_velocity.z = float(gyro[2])
    msg.linear_acceleration.x = float(acc[0])
    msg.linear_acceleration.y = float(acc[1])
    msg.linear_acceleration.z = float(acc[2])
    msg.orientation_covariance[0] = 1e-4
    msg.orientation_covariance[4] = 1e-4
    msg.orientation_covariance[8] = 1e-4
    msg.angular_velocity_covariance[0] = 5e-4
    msg.angular_velocity_covariance[4] = 5e-4
    msg.angular_velocity_covariance[8] = 5e-4
    msg.linear_acceleration_covariance[0] = 1e-2
    msg.linear_acceleration_covariance[4] = 1e-2
    msg.linear_acceleration_covariance[8] = 1e-2
    return msg


def apply_real_mavros_imu_covariance(msg: Any) -> None:
    """Apply median covariance from the April 1 /mavros/imu/data bag."""

    for idx in (0, 4, 8):
        msg.orientation_covariance[idx] = 1e-4
        msg.angular_velocity_covariance[idx] = 1.2184700254281e-7
        msg.linear_acceleration_covariance[idx] = 9.0e-8


__all__ = ["apply_real_mavros_imu_covariance", "build_imu_msg"]

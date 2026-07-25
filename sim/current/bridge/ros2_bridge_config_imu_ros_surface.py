"""ROS-facing IMU acceleration surface calibration."""

from __future__ import annotations

import numpy as np


def configure_ros_imu_accel_surface_contract(bridge: object) -> None:
    bridge._ros_imu_accel_scale = np.array(
        [
            bridge._env_to_clamped_float("ROS2_UUV_ROS_IMU_ACCEL_X_SCALE", 1.0, 0.5, 1.5),
            bridge._env_to_clamped_float("ROS2_UUV_ROS_IMU_ACCEL_Y_SCALE", 1.0, 0.5, 1.5),
            bridge._env_to_clamped_float("ROS2_UUV_ROS_IMU_ACCEL_Z_SCALE", 1.0, 0.5, 1.5),
        ],
        dtype=np.float64,
    )
    bridge._ros_imu_accel_bias = np.array(
        [
            bridge._env_to_clamped_float("ROS2_UUV_ROS_IMU_ACCEL_X_BIAS", 0.0, -5.0, 5.0),
            bridge._env_to_clamped_float("ROS2_UUV_ROS_IMU_ACCEL_Y_BIAS", 0.0, -5.0, 5.0),
            bridge._env_to_clamped_float("ROS2_UUV_ROS_IMU_ACCEL_Z_BIAS", 0.0, -5.0, 5.0),
        ],
        dtype=np.float64,
    )
    if (
        np.max(np.abs(bridge._ros_imu_accel_scale - 1.0)) > 1.0e-9
        or np.max(np.abs(bridge._ros_imu_accel_bias)) > 1.0e-9
    ):
        print(
            "[sensor] ROS IMU accel surface calibration: "
            f"scale={bridge._ros_imu_accel_scale.tolist()}, "
            f"bias={bridge._ros_imu_accel_bias.tolist()}",
            flush=True,
        )


__all__ = ["configure_ros_imu_accel_surface_contract"]

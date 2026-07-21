"""SITL JSON IMU acceleration contract configuration."""

from __future__ import annotations

import numpy as np

from bridge.sitl_env import env_to_float


def configure_sitl_imu_accel_contract(bridge: object) -> None:
    bridge._sitl_accel_xy_sign = float(
        np.clip(env_to_float("ROS2_UUV_SITL_ACCEL_XY_SIGN", 1.0), -1.0, 1.0)
    )
    bridge._sitl_accel_z_sign = float(
        np.clip(env_to_float("ROS2_UUV_SITL_ACCEL_Z_SIGN", 1.0), -1.0, 1.0)
    )
    bridge._imu_accel_xy_scale = bridge._env_to_clamped_float(
        "ROS2_UUV_IMU_ACCEL_XY_SCALE",
        1.0,
        0.5,
        1.5,
    )
    bridge._imu_accel_z_scale = bridge._env_to_clamped_float(
        "ROS2_UUV_IMU_ACCEL_Z_SCALE",
        1.0,
        0.5,
        1.5,
    )
    if (
        abs(bridge._imu_accel_xy_scale - 1.0) > 1.0e-9
        or abs(bridge._imu_accel_z_scale - 1.0) > 1.0e-9
    ):
        print(
            "[sensor] IMU accel scale: "
            f"xy={bridge._imu_accel_xy_scale:.6f}, z={bridge._imu_accel_z_scale:.6f}",
            flush=True,
        )
    if bridge._sitl_accel_z_sign < 0.0:
        print(
            "[sitl] warning: ROS2_UUV_SITL_ACCEL_Z_SIGN=-1 active "
            "(inverts ArduPilot JSON accel_body.z and can initialize attitude upside-down)",
            flush=True,
        )


__all__ = ["configure_sitl_imu_accel_contract"]

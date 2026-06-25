"""IMU acceleration source selection for Ros2Bridge."""

from __future__ import annotations

import os


VALID_IMU_ACCEL_SOURCES = {"mujoco_sensor", "finite_difference"}


def configure_imu_accel_source(bridge: object) -> None:
    bridge._imu_acc_clip_mps2 = 16.0 * bridge._bar30_gravity
    bridge._imu_accel_source = str(
        os.environ.get("ROS2_UUV_IMU_ACCEL_SOURCE", "mujoco_sensor")
    ).strip().lower()
    if bridge._imu_accel_source not in VALID_IMU_ACCEL_SOURCES:
        print(
            f"[sensor] invalid ROS2_UUV_IMU_ACCEL_SOURCE={bridge._imu_accel_source!r}; "
            "using mujoco_sensor",
            flush=True,
        )
        bridge._imu_accel_source = "mujoco_sensor"
    print(f"[sensor] IMU accel source: {bridge._imu_accel_source}", flush=True)


__all__ = ["VALID_IMU_ACCEL_SOURCES", "configure_imu_accel_source"]

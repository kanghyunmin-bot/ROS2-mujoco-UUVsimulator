"""IMU and DVL vector builders for SITL sensor snapshots."""

from __future__ import annotations

from typing import Any

from .ros2_sitl_sensor_altitude import dvl_altitude_from_sensor_or_model
from .ros2_sitl_sensor_dvl_state import dvl_velocity_from_snapshot
from .ros2_sitl_sensor_imu_state import imu_body_vectors_from_snapshot
from .ros2_sitl_sensor_types import BaseKinematicState, ImuDvlState


def build_imu_dvl_state(self, data: Any, base: BaseKinematicState) -> ImuDvlState:
    gyro = self._sensor_slice(self.model, self.sensor_ids, "imu_gyro", data)
    acc_sensor = self._sensor_slice(self.model, self.sensor_ids, "imu_acc", data)
    dvl_vel_sensor = self._sensor_slice(self.model, self.sensor_ids, "dvl_vel_body", data)
    dvl_altitude_m = dvl_altitude_from_sensor_or_model(self, data)
    gyro_bmj, acc_bmj = imu_body_vectors_from_snapshot(self, data, base, gyro, acc_sensor)
    dvl_vel_body_bmj = dvl_velocity_from_snapshot(self, data, base, dvl_vel_sensor, gyro_bmj)

    return ImuDvlState(
        gyro_bmj=gyro_bmj,
        acc_bmj=acc_bmj,
        dvl_vel_body_bmj=dvl_vel_body_bmj,
        dvl_altitude_m=dvl_altitude_m,
    )


__all__ = ["build_imu_dvl_state"]

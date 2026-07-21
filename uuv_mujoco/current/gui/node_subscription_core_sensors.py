"""Core IMU and battery subscriptions for the GUI ROS node."""

from __future__ import annotations

from .runtime import BatteryState, Imu, qos_profile_sensor_data


def initialize_core_sensor_subscriptions(self, *, best_effort_qos) -> None:
    self.create_subscription(Imu, self._topic("imu/data"), self._on_imu, qos_profile_sensor_data)
    self.create_subscription(Imu, "/imu/data", self._on_imu, qos_profile_sensor_data)
    self.create_subscription(BatteryState, self._topic("battery"), self._on_battery, best_effort_qos)
    self.create_subscription(BatteryState, "/battery", self._on_battery, best_effort_qos)


__all__ = ["initialize_core_sensor_subscriptions"]

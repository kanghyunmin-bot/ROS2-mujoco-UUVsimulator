"""ROS2 bridge environment and rate helper methods."""

from __future__ import annotations

import numpy as np

from .sitl_env import env_to_float


def env_to_rate_hz(self, env_name: str, default: float) -> float:
    value = float(env_to_float(env_name, default))
    if value <= 0.0:
        return 0.0
    return float(np.clip(value, 0.01, 500.0))


def env_to_clamped_float(self, env_name: str, default: float, min_value: float, max_value: float) -> float:
    return float(np.clip(env_to_float(env_name, default), min_value, max_value))


def ros_topic_due(self, label: str, sim_t: float, hz: float) -> bool:
    rate = float(hz)
    if rate <= 0.0:
        return True
    period_s = 1.0 / rate
    next_t = self._ros_sensor_rate_next_t.get(label)
    if next_t is None:
        next_t = float(sim_t)
    if sim_t + 1.0e-9 < next_t:
        return False
    while next_t <= sim_t + 1.0e-9:
        next_t += period_s
    self._ros_sensor_rate_next_t[label] = next_t
    return True


def ros_imu_accel_surface(self, acc_ros: np.ndarray) -> np.ndarray:
    acc = np.asarray(acc_ros, dtype=np.float64)
    return np.nan_to_num(
        (acc * self._ros_imu_accel_scale) + self._ros_imu_accel_bias,
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )


__all__ = [
    "env_to_clamped_float",
    "env_to_rate_hz",
    "ros_imu_accel_surface",
    "ros_topic_due",
]

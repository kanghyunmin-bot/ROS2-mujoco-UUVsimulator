"""Servo/status public API helpers for Ros2Bridge."""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np

from .ros2_dvl_sensor_runtime import discard_dvl_scene_velocity, reset_dvl_dead_reckoning
from .ros2_dvl_device_emulator_runtime import (
    reset_dvl_device_emulator_dead_reckoning,
    reset_dvl_device_emulator_report_schedule,
)


def set_sitl_servo_handler(self, callback: Optional[Callable[[list[int]], None]]) -> None:
    if self._sitl_transport is not None:
        def wrapped_servo_handler(pwm_values: list[int]) -> None:
            if callback is not None:
                callback(pwm_values)

        set_telemetry_handler = getattr(self._sitl_transport, "set_servo_telemetry_handler", None)
        if callable(set_telemetry_handler):
            set_telemetry_handler(self._on_sitl_servo_output_for_ros)
        self._sitl_transport.set_servo_handler(wrapped_servo_handler)


def set_replay_rcout_handler(self, callback: Optional[Callable[[list[int]], None]]) -> None:
    self._replay_rcout_handler = callback


def sitl_vehicle_armed(self) -> bool:
    if self._sitl_transport is None:
        return False
    return bool(self._sitl_transport.vehicle_armed)


def sitl_vehicle_mode(self) -> str:
    if self._sitl_transport is None:
        return ""
    return str(self._sitl_transport.vehicle_mode or "")


def force_next_publish(self) -> None:
    """Make the next publish call emit a fresh contract snapshot.

    Plant replay releases a pinned real-start state through a ROS service.
    The first sensor sample after that service must represent the injected
    state, not the last rate-limited pre-release sample.
    """
    self.last_pub_t = -1.0
    self._ros_sensor_rate_next_t.clear()


def reset_odometry(self, *, scene_reset: bool = False) -> None:
    """Reset pose origins and discard queued velocity when restoring the scene."""
    self._odom_pos = np.zeros(3, dtype=np.float64)
    self._last_odom_time = -1.0
    reset_dvl_device_emulator_dead_reckoning(self, scene_reset=scene_reset)
    reset_dvl_dead_reckoning(self)
    if scene_reset:
        discard_dvl_scene_velocity(self)
    reset_dvl_device_emulator_report_schedule(self)
    vision = getattr(self, "_vision_sensor_pose_runtime", None)
    if vision is not None:
        vision.reset_initial()


__all__ = [
    "force_next_publish",
    "reset_odometry",
    "set_replay_rcout_handler",
    "set_sitl_servo_handler",
    "sitl_vehicle_armed",
    "sitl_vehicle_mode",
]

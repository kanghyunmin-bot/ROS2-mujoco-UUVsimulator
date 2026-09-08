"""ROS message helpers for timed IMU and Bar30 deliveries."""

from __future__ import annotations

import numpy as np

from .ros2_bridge_publish_stamp import stamp_from_seconds_like
from .ros2_standard_messages import build_imu_msg, build_pressure_msg


def build_delivered_imu_msg(bridge, stamp, delivery, *, frame_id: str):
    """Build an IMU message stamped at sensor capture time."""

    sample = delivery.sample
    accel = np.asarray(sample.linear_acceleration_mps2, dtype=np.float64)
    accel_surface = getattr(bridge, "_ros_imu_accel_surface", None)
    if callable(accel_surface):
        accel = np.asarray(accel_surface(accel), dtype=np.float64)
    msg = build_imu_msg(
        bridge.Imu,
        stamp_from_seconds_like(stamp, delivery.capture_time_s),
        np.asarray(sample.orientation_wxyz, dtype=np.float64),
        np.asarray(sample.angular_velocity_rad_s, dtype=np.float64),
        accel,
        frame_id=frame_id,
    )
    _set_diagonal(msg.orientation_covariance, sample.orientation_covariance_diag_rad2)
    _set_diagonal(
        msg.angular_velocity_covariance,
        sample.angular_velocity_covariance_diag_rad2_s2,
    )
    _set_diagonal(
        msg.linear_acceleration_covariance,
        sample.linear_acceleration_covariance_diag_m2_s4,
    )
    return msg


def build_delivered_raw_imu_msg(bridge, stamp, delivery, *, frame_id: str):
    """Build a raw IMU message stamped at sensor capture time.

    MAVROS ``data_raw`` does not carry an orientation estimate.  Match that
    physical boundary explicitly instead of leaking the simulator's modeled
    ground-truth quaternion into the raw sensor stream.
    """

    sample = delivery.sample
    accel = np.asarray(sample.linear_acceleration_mps2, dtype=np.float64)
    accel_surface = getattr(bridge, "_ros_imu_accel_surface", None)
    if callable(accel_surface):
        accel = np.asarray(accel_surface(accel), dtype=np.float64)
    msg = build_imu_msg(
        bridge.Imu,
        stamp_from_seconds_like(stamp, delivery.capture_time_s),
        np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float64),
        np.asarray(sample.angular_velocity_rad_s, dtype=np.float64),
        accel,
        frame_id=frame_id,
    )
    for index in range(len(msg.orientation_covariance)):
        msg.orientation_covariance[index] = 0.0
    msg.orientation_covariance[0] = -1.0
    _set_diagonal(
        msg.angular_velocity_covariance,
        sample.angular_velocity_covariance_diag_rad2_s2,
    )
    _set_diagonal(
        msg.linear_acceleration_covariance,
        sample.linear_acceleration_covariance_diag_m2_s4,
    )
    return msg


def build_delivered_pressure_msg(bridge, stamp, delivery, *, frame_id: str):
    """Build a FluidPressure message stamped at sensor capture time."""

    sample = delivery.sample
    return build_pressure_msg(
        bridge.FluidPressure,
        stamp_from_seconds_like(stamp, delivery.capture_time_s),
        sample.measured_pressure_pa,
        frame_id=frame_id,
        variance_pa2=sample.variance_pa2,
    )


def delivered_depth_m(bridge, delivery) -> float:
    """Convert a delivered pressure sample through the configured real law."""

    return float(
        bridge._baro_pressure_law.frontend_depth_m_from_pressure(
            delivery.sample.measured_pressure_pa
        )
    )


def _set_diagonal(covariance, diagonal) -> None:
    for index, value in zip((0, 4, 8), diagonal):
        covariance[index] = float(value)


__all__ = [
    "build_delivered_imu_msg",
    "build_delivered_raw_imu_msg",
    "build_delivered_pressure_msg",
    "delivered_depth_m",
]

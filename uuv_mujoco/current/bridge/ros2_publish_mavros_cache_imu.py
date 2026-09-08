"""MAVROS-compatible IMU message builders."""

from __future__ import annotations

from .ros2_imu_bar30_messages import (
    build_delivered_imu_msg,
    build_delivered_raw_imu_msg,
)
from .ros2_standard_messages import apply_real_mavros_imu_covariance, build_imu_msg


def build_mavros_imu_msg(bridge, stamp, state):
    # The April 1 real bag uses frame_id=fcu_link, but MAVROS has already
    # converted IMU vectors into ROS FLU convention.
    if bool(getattr(bridge, "_imu_sensor_model_enabled", False)):
        if state.imu_sensor_delivery is None:
            return None
        return build_delivered_imu_msg(
            bridge,
            stamp,
            state.imu_sensor_delivery,
            frame_id="fcu_link",
        )
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
    if bool(getattr(bridge, "_imu_sensor_model_enabled", False)):
        if state.imu_sensor_delivery is None:
            return None
        if bool(getattr(bridge, "_strict_sitl_sensor_transport", False)):
            return build_delivered_raw_imu_msg(
                bridge,
                stamp,
                state.imu_sensor_delivery,
                frame_id="fcu_link",
            )
        return build_delivered_imu_msg(
            bridge,
            stamp,
            state.imu_sensor_delivery,
            frame_id="fcu_link",
        )
    msg = build_imu_msg(
        bridge.Imu,
        stamp,
        state.quat_ros,
        state.gyro_ros,
        state.acc_ros_surface,
        frame_id="fcu_link",
    )
    apply_real_mavros_imu_covariance(msg)
    if bool(getattr(bridge, "_strict_sitl_sensor_transport", False)):
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        for index in range(len(msg.orientation_covariance)):
            msg.orientation_covariance[index] = 0.0
        msg.orientation_covariance[0] = -1.0
    return msg


__all__ = ["build_imu_raw_msg", "build_mavros_imu_msg"]

"""Factory registry for lazy MAVROS-compatible publish message cache."""

from __future__ import annotations

from .ros2_publish_mavros_cache_imu import build_imu_raw_msg, build_mavros_imu_msg
from .ros2_publish_mavros_cache_local import (
    build_local_odom_msg,
    build_local_pose_msg,
    build_local_vel_body_cov_msg,
    build_local_vel_body_msg,
    build_local_vel_msg,
    build_vision_pose_msg,
)
from .ros2_publish_mavros_cache_status import (
    build_atm_pressure_msg,
    build_battery_status_msg,
    build_mavros_state_msg,
    build_static_pressure_msg,
    build_vfr_hud_status_msg,
)


def _mavros_state(bridge, stamp, state):
    return build_mavros_state_msg(bridge, stamp)


def _mavros_atm_pressure(bridge, stamp, state):
    return build_atm_pressure_msg(bridge, stamp)


def _mavros_battery(bridge, stamp, state):
    return build_battery_status_msg(bridge, stamp)


MAVROS_CACHE_FACTORIES = (
    ("mavros_state", _mavros_state),
    ("mavros_vfr_hud", build_vfr_hud_status_msg),
    ("mavros_imu", build_mavros_imu_msg),
    ("mavros_imu_raw", build_imu_raw_msg),
    ("mavros_static_pressure", build_static_pressure_msg),
    ("mavros_atm_pressure", _mavros_atm_pressure),
    ("mavros_battery", _mavros_battery),
    ("mavros_local_pose", build_local_pose_msg),
    ("mavros_local_vel", build_local_vel_msg),
    ("mavros_local_vel_body", build_local_vel_body_msg),
    ("mavros_local_vel_body_cov", build_local_vel_body_cov_msg),
    ("mavros_local_odom", build_local_odom_msg),
    ("mavros_vision_pose", build_vision_pose_msg),
)


__all__ = ["MAVROS_CACHE_FACTORIES"]

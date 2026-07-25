"""Base ExternalNav configuration values for SitlTransport."""

from __future__ import annotations

import numpy as np

from bridge.sitl_env import env_flag, env_to_float


def initialize_extnav_base_state(transport: object) -> None:
    transport._sitl_bridge_extnav_disabled = env_flag("ROS2_UUV_SITL_BRIDGE_EXTNAV_DISABLE", False)
    transport._sitl_extnav_enabled = False if transport._sitl_bridge_extnav_disabled else env_flag(
        "ROS2_UUV_SITL_EXTNAV_ENABLE",
        env_flag("SITL_EKF3_EXTNAV", False),
    )
    transport._sitl_extnav_rate_hz = float(np.clip(env_to_float("ROS2_UUV_SITL_EXTNAV_HZ", 10.0), 5.0, 50.0))
    transport._sitl_vpd_confidence = float(np.clip(env_to_float("ROS2_UUV_SITL_VPD_CONFIDENCE", 100.0), 0.0, 100.0))


def initialize_extnav_origin_state(transport: object) -> None:
    transport._sitl_extnav_origin_lat_e7 = int(round(env_to_float("ROS2_UUV_SITL_EXTNAV_ORIGIN_LAT", 47.607584) * 1.0e7))
    transport._sitl_extnav_origin_lon_e7 = int(round(env_to_float("ROS2_UUV_SITL_EXTNAV_ORIGIN_LON", -122.343911) * 1.0e7))
    transport._sitl_extnav_origin_alt_mm = int(round(env_to_float("ROS2_UUV_SITL_EXTNAV_ORIGIN_ALT_M", 0.0) * 1000.0))


def initialize_extnav_vpd_history(transport: object) -> None:
    transport._sitl_vpd_prev_clock_t: float | None = None
    transport._sitl_vpd_prev_pos_ned: np.ndarray | None = None
    transport._sitl_vpd_prev_rot_ned_bfrd: np.ndarray | None = None


__all__ = [
    "initialize_extnav_base_state",
    "initialize_extnav_origin_state",
    "initialize_extnav_vpd_history",
]

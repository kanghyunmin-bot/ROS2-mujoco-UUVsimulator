"""Base runtime-state setup for Ros2Bridge construction."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import numpy as np

from .ros2_bridge_config import (
    configure_dvl_and_frame_transforms,
    configure_mavros_rc_contract,
    configure_mavros_setpoint_contract,
    configure_mavros_state_and_rates,
    configure_pressure_vertical_contract,
)
from .ros2_stereo_image import configure_stereo_image_runtime
from .sitl_env import env_to_float, env_to_int


def configure_bridge_runtime_state(
    bridge: Any,
    *,
    model: Any,
    command_callback: Callable[[float, float, float, float], None],
    cmd_limit: float,
    publish_images: bool,
    image_width: int,
    image_height: int,
    sensor_hz: float,
    image_hz: float,
    camera_calib_left: str,
    camera_calib_right: str,
    enable_sitl: bool,
    enable_ros: bool,
    enable_mavros_surface: bool,
    real_pkg_compat: bool,
) -> None:
    """Initialize constructor state that is independent from ROS message types."""

    bridge._legacy_image_request = False
    bridge.model = model
    configure_stereo_image_runtime(
        bridge,
        publish_images=publish_images,
        image_width=image_width,
        image_height=image_height,
        image_hz=image_hz,
    )
    del camera_calib_left, camera_calib_right

    bridge.command_callback = command_callback
    bridge.cmd_limit = float(cmd_limit)
    bridge.sensor_dt = 1.0 / max(float(sensor_hz), 1e-6)
    bridge.next_sensor_t = 0.0
    bridge.last_pub_t = -1.0
    bridge._ros_spin_period_s = 1.0 / max(env_to_float("ROS2_UUV_SPIN_HZ", 400.0), 1.0)
    bridge._ros_spin_max_callbacks = int(
        np.clip(env_to_int("ROS2_UUV_SPIN_MAX_CALLBACKS", 8), 1, 256)
    )
    bridge._ros_next_spin_wall = 0.0
    bridge.enable_sitl = bool(enable_sitl)
    bridge._enable_ros = bool(enable_ros)
    bridge._mavros_surface_enabled = bool(enable_mavros_surface)
    bridge._real_pkg_compat = bool(real_pkg_compat)
    if bridge._real_pkg_compat and bridge._mavros_surface_enabled:
        raise ValueError("real package compatibility cannot enable the simulator MAVROS surface")
    bridge._ros_ok = False
    bridge._ros_error_reported = False


def configure_command_runtime_state(bridge: Any) -> None:
    bridge.cmd_timeout_s = float(np.clip(env_to_float("ROS2_UUV_CMD_TIMEOUT_S", 0.45), 0.1, 2.0))
    bridge.last_cmd_wall = -1.0
    bridge.cmd_active = False
    bridge._cmd_filter_norm = np.zeros(4, dtype=np.float64)
    bridge._cmd_filter_t = -1.0
    bridge._cmd_deadband_norm = float(np.clip(env_to_float("ROS2_UUV_CMD_DEADBAND", 0.0), 0.0, 0.2))
    bridge._cmd_slew_rate_norm = float(np.clip(env_to_float("ROS2_UUV_CMD_SLEW_RATE", 200.0), 0.0, 200.0))
    bridge._mavros_forward_arm_mode = bool(env_to_int("ROS2_UUV_MAVROS_FORWARD_ARM_MODE", 1))
    bridge._startup_wall = time.monotonic()
    bridge._arm_mode_boot_guard_s = float(
        np.clip(env_to_float("ROS2_UUV_ARM_MODE_BOOT_GUARD_S", 0.0), 0.0, 120.0)
    )
    bridge._sitl_transport_lock = threading.RLock()
    bridge._mavros_rc_override_cache_lock = threading.RLock()
    bridge._mavros_pending_rc_override_channels = None
    bridge._mavros_pending_rc_override_wall = -1.0
    bridge._mavros_last_forwarded_rc_override_wall = -1.0
    bridge._mavros_rc_override_forward_hz = float(
        np.clip(env_to_float("ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ", 400.0), 1.0, 400.0)
    )
    bridge._mavros_rc_override_forward_period_s = 1.0 / bridge._mavros_rc_override_forward_hz
    bridge._mavros_rc_override_stale_s = float(
        np.clip(env_to_float("ROS2_UUV_MAVROS_RC_OVERRIDE_STALE_S", 3.0), 0.1, 10.0)
    )
    bridge._ros_executor_spin_thread_enabled = bool(
        env_to_int("ROS2_UUV_DEDICATED_SPIN_THREAD", 1)
    )
    bridge._ros_spin_timeout_s = float(
        np.clip(env_to_float("ROS2_UUV_SPIN_TIMEOUT_S", 0.001), 0.0, 0.05)
    )
    bridge._ros_spin_stop = threading.Event()
    bridge._ros_spin_thread: threading.Thread | None = None
    bridge._sitl_poll_thread_enabled = bool(
        env_to_int("ROS2_UUV_DEDICATED_SITL_POLL_THREAD", 1)
    )
    default_sitl_poll_hz = env_to_float(
        "ROS2_UUV_SITL_MAVLINK_POLL_HZ",
        100.0,
    )
    bridge._sitl_poll_thread_hz = float(
        np.clip(env_to_float("ROS2_UUV_SITL_POLL_THREAD_HZ", default_sitl_poll_hz), 5.0, 200.0)
    )
    bridge._sitl_poll_stop = threading.Event()
    bridge._sitl_poll_thread: threading.Thread | None = None
    bridge._sitl_poll_error_reported = False


def configure_bridge_contracts(bridge: Any) -> None:
    """Apply MAVROS, pressure, and frame contracts in the original order."""

    configure_mavros_rc_contract(bridge)
    configure_mavros_setpoint_contract(bridge)
    configure_mavros_state_and_rates(bridge)
    configure_pressure_vertical_contract(bridge)
    configure_dvl_and_frame_transforms(bridge)


__all__ = [
    "configure_bridge_contracts",
    "configure_bridge_runtime_state",
    "configure_command_runtime_state",
]

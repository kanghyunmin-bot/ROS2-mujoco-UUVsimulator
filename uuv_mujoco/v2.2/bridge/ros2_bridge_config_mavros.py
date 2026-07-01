"""MAVROS-compatible surface configuration for Ros2Bridge."""

from __future__ import annotations

import os

import numpy as np

from bridge.ros2_bridge_config_mavros_rates import configure_mavros_state_and_rates
from bridge.ros2_math import clamp_rc_channel
from bridge.sitl_env import env_to_float, env_to_int


def configure_mavros_rc_contract(bridge: object) -> None:
    bridge._mavros_rc_forward_channel = clamp_rc_channel(
        env_to_int("ROS2_UUV_MAVROS_RC_CH_FORWARD", 5) - 1
    )
    bridge._mavros_rc_sway_channel = clamp_rc_channel(
        env_to_int("ROS2_UUV_MAVROS_RC_CH_SWAY", 6) - 1
    )
    bridge._mavros_rc_yaw_channel = clamp_rc_channel(
        env_to_int("ROS2_UUV_MAVROS_RC_CH_YAW", 4) - 1
    )
    bridge._mavros_rc_heave_channel = clamp_rc_channel(
        env_to_int("ROS2_UUV_MAVROS_RC_CH_HEAVE", 3) - 1
    )
    bridge._mavros_rc_forward_invert = bool(env_to_int("ROS2_UUV_MAVROS_RC_INV_FORWARD", 0))
    bridge._mavros_rc_sway_invert = bool(env_to_int("ROS2_UUV_MAVROS_RC_INV_SWAY", 0))
    bridge._mavros_rc_yaw_invert = bool(env_to_int("ROS2_UUV_MAVROS_RC_INV_YAW", 0))
    bridge._mavros_rc_heave_invert = bool(env_to_int("ROS2_UUV_MAVROS_RC_INV_HEAVE", 1))
    bridge._mavros_rc_pwm_span = float(
        np.clip(env_to_float("ROS2_UUV_MAVROS_RC_PWM_SPAN", 300.0), 50.0, 700.0)
    )
    bridge._mavros_rc_override_local_fallback = bool(
        env_to_int("ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK", 0)
    )
    bridge._mavros_rc_override_backend = _rc_override_backend_from_env()
    bridge._sitl_allow_direct_cmd = bool(env_to_int("ROS2_UUV_SITL_ALLOW_DIRECT_CMD", 0))
    bridge._sitl_cmd_vel_setpoint_enabled = bool(
        env_to_int("ROS2_UUV_SITL_CMD_VEL_SETPOINT_ENABLE", 0)
    )
    bridge._sitl_cmd_vel_blocked_warned = False
    bridge._sitl_direct_cmd_blocked_warned = False
    bridge._allow_rcout_plant_override = bool(
        env_to_int("ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE", 0)
    )


def _rc_override_backend_from_env() -> str:
    backend = os.environ.get(
        "ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND",
        "rc_override",
    ).strip().lower().replace("-", "_")
    if backend in {"manual_control", "manual"}:
        return "manual_control"
    if backend in {"rc_channels_override", "rc_override", "mavlink_rc_override", "raw"}:
        return "rc_channels_override"
    print(
        "[ros2_bridge] warning: unknown ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND="
        f"{backend!r}; using rc_override",
        flush=True,
    )
    return "rc_channels_override"


def configure_mavros_setpoint_contract(bridge: object) -> None:
    bridge._mavros_setpoint_enabled = bool(env_to_int("ROS2_UUV_MAVROS_SETPOINT_ENABLE", 0))
    bridge._mavros_setpoint_pos_kp = float(env_to_float("ROS2_UUV_MAVROS_SETPOINT_POS_KP", 0.55))
    bridge._mavros_setpoint_heave_kp = float(
        env_to_float("ROS2_UUV_MAVROS_SETPOINT_HEAVE_KP", 0.55)
    )
    bridge._mavros_setpoint_yaw_kp = float(env_to_float("ROS2_UUV_MAVROS_SETPOINT_YAW_KP", 1.2))
    bridge._mavros_setpoint_timeout_s = float(
        env_to_float("ROS2_UUV_MAVROS_SETPOINT_TIMEOUT_S", 1.0)
    )
    bridge._mavros_setpoint_last_t = -1.0
    bridge._mavros_setpoint_pos = None
    bridge._mavros_setpoint_yaw = None
    bridge._mavros_pending_yaw_delta = 0.0


__all__ = [
    "configure_mavros_rc_contract",
    "configure_mavros_setpoint_contract",
    "configure_mavros_state_and_rates",
]

"""Forced GUI-start command and readiness environment contract."""

from __future__ import annotations

from typing import Mapping


def _command_rate_defaults(env: Mapping[str, str]) -> dict[str, str]:
    profile = str(env.get("UUV_RUNTIME_PROFILE", "balanced")).strip().lower()
    if profile == "low":
        return {
            "ROS2_UUV_SPIN_HZ": "80",
            "ROS2_UUV_SITL_COMMAND_POLL_HZ": "80",
            "ROS2_UUV_SITL_MAVLINK_POLL_HZ": "30",
            "ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ": "80",
        }
    return {
        "ROS2_UUV_SPIN_HZ": "60",
        "ROS2_UUV_SITL_COMMAND_POLL_HZ": "80",
        "ROS2_UUV_SITL_MAVLINK_POLL_HZ": "40",
        "ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ": "120",
    }


def command_readiness_contract(env: Mapping[str, str]) -> dict[str, str]:
    rate_defaults = _command_rate_defaults(env)
    return {
        # QGC is the primary operator path. Keep auto-arm/mode bootstrap
        # opt-in so startup readiness does not depend on a second command
        # MAVLink path.
        "SITL_AUTO_SAFE_SEQUENCE": "0",
        "ROS2_UUV_SITL_AUTO_READY": env.get("ROS2_UUV_SITL_AUTO_READY", "0"),
        "ROS2_UUV_SITL_AUTO_READY_MODE": "MANUAL",
        # Explicit GUI/QGC/test arm/mode commands must still reach ArduSub.
        "ROS2_UUV_MAVROS_FORWARD_ARM_MODE": "1",
        "ROS2_UUV_DEDICATED_SPIN_THREAD": "1",
        "ROS2_UUV_SPIN_HZ": env.get("ROS2_UUV_SPIN_HZ", rate_defaults["ROS2_UUV_SPIN_HZ"]),
        "ROS2_UUV_SPIN_TIMEOUT_S": env.get("ROS2_UUV_SPIN_TIMEOUT_S", "0.001"),
        "ROS2_UUV_SITL_COMMAND_POLL_HZ": env.get(
            "ROS2_UUV_SITL_COMMAND_POLL_HZ",
            rate_defaults["ROS2_UUV_SITL_COMMAND_POLL_HZ"],
        ),
        "ROS2_UUV_SITL_MAVLINK_POLL_HZ": env.get(
            "ROS2_UUV_SITL_MAVLINK_POLL_HZ",
            rate_defaults["ROS2_UUV_SITL_MAVLINK_POLL_HZ"],
        ),
        "ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ": env.get(
            "ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ",
            rate_defaults["ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ"],
        ),
        "ROS2_UUV_CMD_TIMEOUT_S": env.get("ROS2_UUV_CMD_TIMEOUT_S", "0.25"),
        "ROS2_UUV_CMD_DEADBAND": env.get("ROS2_UUV_CMD_DEADBAND", "0.0"),
        "ROS2_UUV_CMD_SLEW_RATE": env.get("ROS2_UUV_CMD_SLEW_RATE", "0"),
        "UUV_GUI_ARM_MODE_COMMAND_PATH": "topic",
        "UUV_GUI_CONTROL_REQUEST_TIMEOUT_S": env.get("UUV_GUI_CONTROL_REQUEST_TIMEOUT_S", "15.0"),
        "UUV_GUI_CONTROL_REQUEST_RETRY_S": env.get("UUV_GUI_CONTROL_REQUEST_RETRY_S", "0.05"),
        # Ready must mean the operator can command immediately. Extra EKF settle is
        # explicit A/B diagnostics only, never a hidden GUI-start delay.
        "UUV_GUI_REQUIRE_ARM_MODE_EKF_SETTLE": "0",
        "UUV_GUI_ARM_MODE_EKF_SETTLE_S": "0.0",
        "ROS2_UUV_SITL_ALLOW_DIRECT_CMD": env.get("ROS2_UUV_SITL_ALLOW_DIRECT_CMD", "1"),
        "ROS2_UUV_SITL_CMD_VEL_SETPOINT_ENABLE": "0",
        "ROS2_UUV_MAVROS_SETPOINT_ENABLE": "0",
        "UUV_GUI_PILOT_CONTROL_MODE": env.get("UUV_GUI_PILOT_CONTROL_MODE", "rc_override"),
        "ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND": env.get(
            "ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND",
            "rc_override",
        ),
        "SITL_SCHED_LOOP_RATE": env.get("SITL_SCHED_LOOP_RATE", "400"),
    }


__all__ = ["command_readiness_contract"]

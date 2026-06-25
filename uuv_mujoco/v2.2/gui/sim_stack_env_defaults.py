"""Profile and command-link defaults for GUI-started simulator env."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

from .sim_stack_env_flags import env_bool


def profile_defaults(env: Mapping[str, str], contract_loop_hz: str) -> dict[str, str]:
    defaults = {
        "UUV_RUNTIME_PROFILE": "balanced",
        "SITL_SENSOR_HZ_DEFAULT": "60",
        "SITL_THRUSTER_LOOP_HZ_DEFAULT": "80",
        "UUV_ROS2_SENSOR_HZ": "60",
        "UUV_THRUSTER_LOOP_HZ": "80",
        "UUV_MUJOCO_VIEWER_FPS": "30",
        "UUV_MUJOCO_VIEWER_CAMERA_MODE": "course_overview",
        "ROS2_UUV_SPIN_HZ": "400",
        "ROS2_UUV_DEMAND_PROBE_PERIOD_S": "1.0",
        "ROS2_UUV_SITL_CMD_DEBUG": "0",
        "ROS2_UUV_SITL_MAVLINK_POLL_HZ": "200",
        "ROS2_UUV_SITL_COMMAND_POLL_HZ": "400",
        "SITL_DEDICATED_COMMAND_MAVLINK": "0",
        "ROS2_UUV_COMMAND_LINK_TELEMETRY": "0",
        "ROS2_UUV_COMMAND_LINK_AP_TELEMETRY": "0",
        "ROS2_UUV_ARM_MODE_BOOT_GUARD_S": "0",
    }
    raw_profile = str(env.get("UUV_RUNTIME_PROFILE", "balanced")).strip().lower()
    profile = raw_profile if raw_profile in {"balanced", "low", "high"} else "balanced"
    defaults["UUV_RUNTIME_PROFILE"] = profile
    if profile == "low":
        defaults.update(
            {
                "SITL_SENSOR_HZ_DEFAULT": "30",
                "SITL_THRUSTER_LOOP_HZ_DEFAULT": "60",
                "UUV_ROS2_SENSOR_HZ": "30",
                "UUV_THRUSTER_LOOP_HZ": "60",
                "UUV_MUJOCO_VIEWER_FPS": "30",
                "ROS2_UUV_SPIN_HZ": "100",
                "ROS2_UUV_SITL_MAVLINK_POLL_HZ": "100",
                "ROS2_UUV_SITL_COMMAND_POLL_HZ": "100",
            }
        )
    elif profile == "high":
        defaults.update(
            {
                "SITL_SENSOR_HZ_DEFAULT": "120",
                "SITL_THRUSTER_LOOP_HZ_DEFAULT": "100",
                "UUV_ROS2_SENSOR_HZ": "120",
                "UUV_THRUSTER_LOOP_HZ": "100",
                "UUV_MUJOCO_VIEWER_FPS": "60",
                "ROS2_UUV_SPIN_HZ": "400",
                "ROS2_UUV_SITL_MAVLINK_POLL_HZ": "200",
                "ROS2_UUV_SITL_COMMAND_POLL_HZ": "400",
            }
        )
    return defaults


def apply_command_endpoint_defaults(env: dict[str, str], *, backend: str) -> None:
    contract_loop_hz = env.get("SITL_SCHED_LOOP_RATE", "400")
    command_mav_port = env.get("SITL_COMMAND_MAV_PORT", "14661")
    tcp_mav_port = env.get("SITL_TCP_MAVLINK_PORT", "5760")
    explicit_command_endpoint = "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT" in env
    docker_command_endpoint = env.get(
        "ROS2_UUV_DOCKER_SITL_COMMAND_MAVLINK_ENDPOINT",
        f"udpin:0.0.0.0:{command_mav_port}",
    )
    dedicated_docker_command_endpoint = env.get(
        "ROS2_UUV_DOCKER_SITL_COMMAND_MAVLINK_ENDPOINT",
        f"udpin:0.0.0.0:{command_mav_port}",
    )
    default_command_endpoint = (
        docker_command_endpoint
        if str(backend).strip().lower() == "docker"
        else f"tcp:127.0.0.1:{tcp_mav_port}"
    )
    defaults = profile_defaults(env, contract_loop_hz)
    env["UUV_RUNTIME_PROFILE"] = defaults["UUV_RUNTIME_PROFILE"]
    defaults.update(
        {
            "SITL_COMMAND_MAV_PORT": command_mav_port,
            "SITL_TCP_MAVLINK_PORT": tcp_mav_port,
            "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT": default_command_endpoint,
        }
    )
    for key, value in defaults.items():
        env.setdefault(key, value)

    if str(backend).strip().lower() == "docker" and env_bool(env, "SITL_DEDICATED_COMMAND_MAVLINK", "0"):
        # Optional split command path for A/B debugging. The default keeps
        # commands on the already-open MuJoCo MAVLink session to avoid an
        # extra UDP port and peer-discovery race.
        env["ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT"] = dedicated_docker_command_endpoint
    elif str(backend).strip().lower() == "docker" and not explicit_command_endpoint:
        env["ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT"] = "same"


def apply_native_stable_defaults(
    env: dict[str, str],
    *,
    sim_stack_dir: Path,
    explicit_keys: set[str],
) -> None:
    """Use the macOS native ArduSub stable contract for GUI Start."""

    workspace_dir = sim_stack_dir.resolve().parents[1]
    stable_dir = env.get("ARDUPILOT_STABLE_DIR", str(workspace_dir / "ardupilot_sub_stable"))
    env["ARDUPILOT_DIR"] = stable_dir

    defaults = {
        "SITL_DIRECT_MAVLINK": "1",
        "SITL_QGC_OUTPUT_ENABLE": "1",
        "SITL_QGC_DIRECT_SERIAL_ENABLE": "1",
        "SITL_SERIAL0_UDPCLIENT": "0",
        "SITL_DEDICATED_COMMAND_MAVLINK": "1",
        "SITL_NO_EXTRA_PORTS": "1",
        "SITL_PARAM_COMPAT_FILTER": "1",
        "SITL_USE_REAL_PARAM_FILE": "0",
        "SITL_WIPE_EEPROM": "1",
        "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT": "udpin:0.0.0.0:14661",
    }
    for key, value in defaults.items():
        if key not in explicit_keys:
            env[key] = value


__all__ = ["profile_defaults", "apply_command_endpoint_defaults", "apply_native_stable_defaults"]

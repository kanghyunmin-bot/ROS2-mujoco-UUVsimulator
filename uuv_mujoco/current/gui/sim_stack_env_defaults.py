"""Profile and command-link defaults for GUI-started simulator env."""

from __future__ import annotations

from pathlib import Path
from ctypes.util import find_library
from typing import Mapping

from .sim_stack_env_flags import env_bool


def profile_defaults(env: Mapping[str, str], contract_loop_hz: str) -> dict[str, str]:
    defaults = {
        "UUV_RUNTIME_PROFILE": "balanced",
        # ArduSub's real ATC tuning needs this closed-loop cadence.  A 30 Hz
        # sensor / 40 Hz plant loop adds enough delay to drive roll/pitch into
        # a saturated limit cycle; 100/100 Hz is the validated safe floor.
        "SITL_SENSOR_HZ_DEFAULT": "100",
        "SITL_THRUSTER_LOOP_HZ_DEFAULT": "100",
        "SITL_MAVLINK_SERVO_HZ_DEFAULT": "30",
        # JSON/SERVO PWM is already the final ArduSub motor output.  Feed the
        # raw PWM delta through the measured T200 curve exactly once; an extra
        # scale here raises the closed-loop plant gain and destabilises the
        # real-vehicle attitude tuning.
        "SITL_SERVO_SCALE_DEFAULT": "1.0",
        "SITL_SPEEDUP_DEFAULT": "1",
        "UUV_ROS2_SENSOR_HZ": "100",
        "UUV_THRUSTER_LOOP_HZ": "100",
        # 8 ms is not contact-stable for the 10 g course buoys: sequential
        # collector capture exceeds the physical slot tolerance.  Five
        # milliseconds is the measured safe ceiling and the runtime guard
        # prevents an external profile from silently exceeding it whenever a
        # course-buoy scene is loaded.
        "UUV_MUJOCO_TIMESTEP": "0.005",
        "UUV_COURSE_BUOY_TIMESTEP_GUARD": "1",
        "UUV_COURSE_BUOY_UPDATE_HZ": "10",
        "UUV_COURSE_BUOY_TRACK_CSV_ENABLE": "0",
        "UUV_COURSE_BUOY_TRACK_CSV_INTERVAL_S": "0.50",
        "UUV_MUJOCO_VIEWER_FPS": "30",
        "UUV_MUJOCO_VIEWER_WIDTH": "1280",
        "UUV_MUJOCO_VIEWER_HEIGHT": "720",
        "UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP": "0",
        "UUV_MUJOCO_VIEWER_MAX_CATCHUP_STEPS": "2",
        "UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC": "1",
        "UUV_MUJOCO_SHADOW_SIZE": "1024",
        "UUV_MUJOCO_OFFSAMPLES": "1",
        "UUV_GUI_MUJOCO_VIEWER": "1",
        "UUV_MUJOCO_VIEWER_CAMERA_MODE": "course_side",
        "UUV_MUJOCO_CATCHUP_WINDOW_S": "0.040",
        "UUV_MUJOCO_SENSOR_CATCHUP_WINDOW_S": "0.067",
        "UUV_MUJOCO_MAX_STEP_LAG_S": "0.040",
        "UUV_MUJOCO_MAX_SENSOR_LAG_S": "0.067",
        "UUV_MUJOCO_MAX_SLEEP_S": "0.001",
        "UUV_MUJOCO_DROP_EXCESS_STEP_LAG": "0",
        "ROS2_UUV_SPIN_HZ": "60",
        "ROS2_UUV_DEMAND_PROBE_PERIOD_S": "1.0",
        "ROS2_UUV_ASYNC_CAMERA_RENDER": "1",
        "UUV_GUI_YOLO_ENABLE": "0",
        "ROS2_UUV_SITL_CMD_DEBUG": "0",
        "ROS2_UUV_SITL_MAVLINK_POLL_HZ": "40",
        "ROS2_UUV_SITL_COMMAND_POLL_HZ": "80",
        "ROS2_UUV_DEDICATED_SITL_POLL_THREAD": "1",
        "ROS2_UUV_SITL_POLL_THREAD_HZ": "40",
        "SITL_DEDICATED_COMMAND_MAVLINK": "0",
        "ROS2_UUV_COMMAND_LINK_TELEMETRY": "0",
        "ROS2_UUV_COMMAND_LINK_AP_TELEMETRY": "0",
        "ROS2_UUV_ARM_MODE_BOOT_GUARD_S": "0",
        "ROS2_UUV_HYDROPHONE_FREQ_HZ": "21164",
        "ROS2_UUV_HYDROPHONE_SAMPLE_RATE_HZ": "96000",
        # Match audio_phase_estimator's 4096-sample windows at 96 kHz. This
        # avoids publishing two or three unstamped delta-range updates in a burst.
        "ROS2_UUV_HYDROPHONE_AUDIO_HZ": "23.4375",
        "ROS2_UUV_HYDROPHONE_SYNC_HZ": "50",
        "ROS2_UUV_HYDROPHONE_STATUS_HZ": "5",
    }
    raw_profile = str(env.get("UUV_RUNTIME_PROFILE", "balanced")).strip().lower()
    profile = raw_profile if raw_profile in {"balanced", "low", "high"} else "balanced"
    defaults["UUV_RUNTIME_PROFILE"] = profile
    if profile == "low":
        defaults.update(
            {
                # Low reduces rendering and auxiliary work, never the
                # STABILIZE/ALT_HOLD feedback cadence.
                "SITL_SENSOR_HZ_DEFAULT": "100",
                "SITL_THRUSTER_LOOP_HZ_DEFAULT": "100",
                "SITL_MAVLINK_SERVO_HZ_DEFAULT": "25",
                "SITL_SERVO_SCALE_DEFAULT": "1.0",
                "SITL_SPEEDUP_DEFAULT": "1",
                "UUV_ROS2_SENSOR_HZ": "100",
                "UUV_THRUSTER_LOOP_HZ": "100",
                "UUV_MUJOCO_TIMESTEP": "0.005",
                "UUV_COURSE_BUOY_UPDATE_HZ": "10",
                "UUV_COURSE_BUOY_TRACK_CSV_INTERVAL_S": "3.00",
                "UUV_MUJOCO_VIEWER_FPS": "20",
                "UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP": "1",
                "UUV_MUJOCO_SHADOW_SIZE": "1024",
                "UUV_MUJOCO_OFFSAMPLES": "1",
                "UUV_MUJOCO_CATCHUP_WINDOW_S": "0.075",
                "UUV_MUJOCO_SENSOR_CATCHUP_WINDOW_S": "0.150",
                "UUV_MUJOCO_MAX_STEP_LAG_S": "0.075",
                "UUV_MUJOCO_MAX_SENSOR_LAG_S": "0.150",
                "UUV_MUJOCO_MAX_SLEEP_S": "0.006",
                "ROS2_UUV_SPIN_HZ": "80",
                "ROS2_UUV_SITL_MAVLINK_POLL_HZ": "30",
                "ROS2_UUV_SITL_COMMAND_POLL_HZ": "80",
                "ROS2_UUV_SITL_POLL_THREAD_HZ": "60",
            }
        )
    elif profile == "high":
        defaults.update(
            {
                "SITL_SENSOR_HZ_DEFAULT": "140",
                "SITL_THRUSTER_LOOP_HZ_DEFAULT": "160",
                "SITL_MAVLINK_SERVO_HZ_DEFAULT": "60",
                "SITL_SERVO_SCALE_DEFAULT": "1.0",
                "SITL_SPEEDUP_DEFAULT": "2",
                "UUV_ROS2_SENSOR_HZ": "140",
                "UUV_THRUSTER_LOOP_HZ": "160",
                "UUV_MUJOCO_TIMESTEP": "0.002",
                "UUV_COURSE_BUOY_UPDATE_HZ": "30",
                "UUV_COURSE_BUOY_TRACK_CSV_INTERVAL_S": "0.25",
                "UUV_MUJOCO_VIEWER_FPS": "30",
                "UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP": "1",
                "UUV_MUJOCO_SHADOW_SIZE": "4096",
                "UUV_MUJOCO_OFFSAMPLES": "4",
                "UUV_MUJOCO_CATCHUP_WINDOW_S": "0.120",
                "UUV_MUJOCO_SENSOR_CATCHUP_WINDOW_S": "0.160",
                "UUV_MUJOCO_MAX_STEP_LAG_S": "0.120",
                "UUV_MUJOCO_MAX_SENSOR_LAG_S": "0.160",
                "UUV_MUJOCO_MAX_SLEEP_S": "0.001",
                "ROS2_UUV_SPIN_HZ": "400",
                "ROS2_UUV_SITL_MAVLINK_POLL_HZ": "200",
                "ROS2_UUV_SITL_COMMAND_POLL_HZ": "400",
                "ROS2_UUV_SITL_POLL_THREAD_HZ": "200",
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


def apply_mujoco_viewer_display_defaults(
    env: dict[str, str],
    *,
    explicit_keys: set[str],
) -> None:
    """Prefer XWayland for MuJoCo's GLFW viewer on Wayland desktops.

    GLFW's native Wayland path can lose client-side decorations depending on
    compositor/libdecor details.  The GUI starts an interactive desktop viewer,
    so default to the X11/XWayland path when both DISPLAY and WAYLAND_DISPLAY
    are available.  Users can opt back into native Wayland with
    UUV_GUI_MUJOCO_XWAYLAND=0.
    """

    if not env_bool(env, "UUV_GUI_MUJOCO_XWAYLAND", "1"):
        return
    if not env.get("DISPLAY") or not env.get("WAYLAND_DISPLAY"):
        return

    if "GLFW_PLATFORM" not in explicit_keys:
        env["GLFW_PLATFORM"] = "x11"
    if "QT_QPA_PLATFORM" not in explicit_keys:
        env["QT_QPA_PLATFORM"] = "xcb"
    if "GDK_BACKEND" not in explicit_keys:
        env["GDK_BACKEND"] = "x11"
    if "SDL_VIDEODRIVER" not in explicit_keys:
        env["SDL_VIDEODRIVER"] = "x11"
    if "PYGLFW_LIBRARY" not in explicit_keys:
        glfw_library = _system_glfw_library()
        if glfw_library:
            env["PYGLFW_LIBRARY"] = glfw_library

    # Removing WAYLAND_DISPLAY is the robust part: it prevents GLFW/MuJoCo from
    # selecting the native Wayland backend even when the session itself is
    # Wayland. DISPLAY remains available for XWayland.
    env.pop("WAYLAND_DISPLAY", None)


def _system_glfw_library() -> str:
    for candidate in (
        "/lib/x86_64-linux-gnu/libglfw.so.3",
        "/usr/lib/x86_64-linux-gnu/libglfw.so.3",
        "/lib/aarch64-linux-gnu/libglfw.so.3",
        "/usr/lib/aarch64-linux-gnu/libglfw.so.3",
    ):
        if Path(candidate).exists():
            return candidate
    return find_library("glfw") or ""


def apply_native_stable_defaults(
    env: dict[str, str],
    *,
    sim_stack_dir: Path,
    explicit_keys: set[str],
) -> None:
    """Use the native ArduSub stable contract for GUI Start."""

    workspace_dir = sim_stack_dir.resolve().parents[1]
    local_libdecor = workspace_dir / ".local_libdecor/usr/lib/x86_64-linux-gnu/libdecor/plugins-1"
    if "LIBDECOR_PLUGIN_DIR" not in explicit_keys and local_libdecor.exists():
        env["LIBDECOR_PLUGIN_DIR"] = str(local_libdecor)

    stable_dir = Path(env.get("ARDUPILOT_STABLE_DIR", str(workspace_dir / "ardupilot_sub_stable")))
    installed_dir = workspace_dir / "ardupilot"
    if "ARDUPILOT_DIR" not in explicit_keys:
        env["ARDUPILOT_DIR"] = str(stable_dir if stable_dir.exists() else installed_dir)

    defaults = {
        "SITL_DIRECT_MAVLINK": "0",
        "SITL_QGC_OUTPUT_ENABLE": "1",
        "SITL_QGC_DIRECT_SERIAL_ENABLE": "0",
        "SITL_SERIAL0_UDPCLIENT": "0",
        "SITL_DEDICATED_COMMAND_MAVLINK": "1",
        "SITL_NO_EXTRA_PORTS": "1",
        "SITL_PARAM_COMPAT_FILTER": "1",
        "SITL_USE_REAL_PARAM_FILE": "0",
        "SITL_WIPE_EEPROM": "1",
        "ROS2_UUV_SITL_JSON_SERVO_FALLBACK": "1",
        "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT": "udpin:0.0.0.0:14661",
    }
    for key, value in defaults.items():
        if key not in explicit_keys:
            env[key] = value


__all__ = [
    "profile_defaults",
    "apply_command_endpoint_defaults",
    "apply_mujoco_viewer_display_defaults",
    "apply_native_stable_defaults",
]

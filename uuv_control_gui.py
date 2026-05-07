#!/usr/bin/env python3
"""Standalone ROS2 GUI for UUV telemetry and basic MAVROS control."""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import re
import shutil
import site
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from tkinter import filedialog, ttk
from typing import Any, Deque, Iterable, Optional


def _sanitize_python_import_path() -> None:
    """Drop site-packages entries from other Python envs before importing ROS."""
    current_prefix = os.path.realpath(sys.prefix)

    def _is_under_current_prefix(path: str) -> bool:
        try:
            return os.path.commonpath([current_prefix, os.path.realpath(path)]) == current_prefix
        except ValueError:
            return False

    def _is_foreign_env_path(path: str) -> bool:
        real_path = os.path.realpath(path)
        if "/miniconda3/envs/" in real_path or "/.venvs/" in real_path:
            return not _is_under_current_prefix(real_path)
        return False

    pythonpath = os.environ.get("PYTHONPATH")
    if pythonpath:
        kept = [p for p in pythonpath.split(os.pathsep) if p and not _is_foreign_env_path(p)]
        if kept:
            os.environ["PYTHONPATH"] = os.pathsep.join(kept)
        else:
            os.environ.pop("PYTHONPATH", None)

    try:
        user_site = os.path.realpath(site.getusersitepackages())
    except Exception:
        user_site = None

    filtered_sys_path = []
    for entry in sys.path:
        if user_site and os.path.realpath(entry) == user_site:
            continue
        if _is_foreign_env_path(entry):
            continue
        filtered_sys_path.append(entry)
    sys.path[:] = filtered_sys_path

    for entry in list(sys.path_importer_cache):
        if _is_foreign_env_path(entry):
            sys.path_importer_cache.pop(entry, None)
        elif user_site and os.path.realpath(entry) == user_site:
            sys.path_importer_cache.pop(entry, None)


_sanitize_python_import_path()

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, FluidPressure, Imu
from std_msgs.msg import Float32, String

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    HAVE_ROSBAG2_PY = True
except ModuleNotFoundError:
    HAVE_ROSBAG2_PY = False
    rosbag2_py = None
    deserialize_message = None

try:
    from mavros_msgs.msg import OverrideRCIn, RCOut, State, StatusText
    from mavros_msgs.srv import CommandBool, SetMode, VehicleInfoGet

    HAVE_MAVROS_MSGS = True
except ModuleNotFoundError:
    HAVE_MAVROS_MSGS = False

    class OverrideRCIn:
        CHAN_RELEASE = 0
        CHAN_NOCHANGE = 65535

        def __init__(self) -> None:
            self.channels = []

    class RCOut:
        def __init__(self) -> None:
            self.channels = []

    class State:
        connected = False
        armed = False
        guided = False
        manual_input = False
        mode = ""
        system_status = 0

    class StatusText:
        EMERGENCY = 0
        ALERT = 1
        CRITICAL = 2
        ERROR = 3
        WARNING = 4
        NOTICE = 5
        INFO = 6
        DEBUG = 7

        def __init__(self) -> None:
            self.severity = self.INFO
            self.text = ""

    class CommandBool:
        class Request:
            def __init__(self) -> None:
                self.value = False

    class SetMode:
        class Request:
            def __init__(self) -> None:
                self.base_mode = 0
                self.custom_mode = ""

    class VehicleInfoGet:
        class Request:
            def __init__(self) -> None:
                self.sysid = 1
                self.compid = 1
                self.get_all = False

BACKEND_AUTO = "auto"
BACKEND_NONE = "none"
BACKEND_MAVROS = "mavros"
BACKEND_SIM_BRIDGE = "sim_bridge"
DEFAULT_AUTO_BACKEND = BACKEND_NONE
APP_ROOT = Path(__file__).resolve().parent

AXIS_MIN = -1.0
AXIS_MAX = 1.0
AXIS_DEADBAND = 0.03
RC_NEUTRAL_PWM = 1500
RC_PWM_SPAN = 300.0
RC_MESSAGE_CHANNEL_COUNT = 18
RC_FEEDBACK_CHANNEL_COUNT = 16
RC_VISIBLE_CHANNEL_COUNT = 8
PRIMARY_RC_CHANNEL_COUNT = 6
TELEMETRY_EVENT_LIMIT = 200
UI_UPDATE_PERIOD_MS = 100
MAX_DEPTH_DISPLAY_M = 2.0
WINDOW_GEOMETRY = "1040x680"
WINDOW_MINSIZE = (900, 560)
TELEMETRY_HIDDEN_WIDTH = 560
TELEMETRY_HIDDEN_MINSIZE = (520, 560)
OUTER_PADDING = 6
GROUP_PADDING = 6
INNER_PADDING = 4
JOYSTICK_CANVAS_SIZE = 130
ATTITUDE_CANVAS_WIDTH = 360
ATTITUDE_CANVAS_HEIGHT = 190
DEPTH_CANVAS_WIDTH = 190
DEPTH_CANVAS_HEIGHT = 96
RC_REPLAY_TOPIC = "/mavros/rc/override"
DEFAULT_RC_REPLAY_BAG = APP_ROOT / (
    "real_robot_ros_bag/extracted_2026_04_01/"
    "bag_2026-04-01_20-08-11_rc_override_no_invert"
)
DEFAULT_AUTOTUNE_BAG = APP_ROOT / (
    "real_robot_ros_bag/extracted_2026_04_01/"
    "bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_0.db3"
)
AUTOTUNE_SCRIPT = APP_ROOT / "document" / "docsource" / "run_uuv_param_autotune.py"
SIM_STACK_DIR = APP_ROOT / "uuv_mujoco" / "v2.2"
START_SIM_STACK_SCRIPT = SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh"
RESET_SIM_STACK_SCRIPT = SIM_STACK_DIR / "reset_uuv_sim.sh"
PING360_RVIZ_CONFIG = APP_ROOT / "rospkg" / "kmu26_auv" / "rviz" / "ping360.rviz"
PHYSICS_PROFILE_PATH = SIM_STACK_DIR / "config" / "sim_profiles.json"
PHYSICS_PROFILE_NAME = "current"
PHYSICS_PARAM_SPECS: tuple[dict[str, Any], ...] = (
    {
        "key": "buoyancy_scale",
        "label": "Buoyancy scale",
        "default": 1.0005,
        "lower": 0.95,
        "upper": 1.05,
        "description": "Net buoyancy multiplier. Small changes strongly affect depth trim.",
    },
    {
        "key": "surface_heave_damping",
        "label": "Surface heave damping",
        "default": 13.2,
        "lower": 0.0,
        "upper": 80.0,
        "description": "Extra vertical damping near the water surface.",
    },
    {
        "key": "heave_damping_scale",
        "label": "Heave damping scale",
        "default": 6.0,
        "lower": 0.0,
        "upper": 20.0,
        "description": "Fully-submerged vertical damping multiplier.",
    },
    {
        "key": "buoyancy_slope_scale",
        "label": "Buoyancy slope scale",
        "default": 1.2,
        "lower": 0.1,
        "upper": 8.0,
        "description": "Waterline buoyancy transition sharpness. High values can make depth response stiff.",
    },
    {
        "key": "cob_torque_scale",
        "label": "CoB torque scale",
        "default": 0.35,
        "lower": 0.0,
        "upper": 2.0,
        "description": "Restoring moment gain from CB-CG offset. Main roll/pitch stability knob.",
    },
    {
        "key": "cob_z_offset",
        "label": "CoB z offset",
        "default": 0.012,
        "lower": -0.05,
        "upper": 0.08,
        "description": "Vertical CB-CG offset. Higher usually increases roll/pitch restoring.",
    },
    {
        "key": "cob_x_offset",
        "label": "CoB x offset",
        "default": 0.009,
        "lower": -0.08,
        "upper": 0.08,
        "description": "Forward CB offset. Tunes pitch trim under buoyancy.",
    },
    {
        "key": "buoyancy_point_blend",
        "label": "Buoyancy point blend",
        "default": 1.0,
        "lower": 0.0,
        "upper": 1.0,
        "description": "0=center buoyancy, 1=distributed buoyancy points. Affects roll/pitch torque.",
    },
    {
        "key": "thruster_force_max",
        "label": "Thruster force max",
        "default": 21.0,
        "lower": 1.0,
        "upper": 100.0,
        "description": "Per-thruster force limit used by the disabled-performance simple model.",
    },
    {
        "key": "linear_drag",
        "label": "Legacy linear drag",
        "default": 0.93,
        "lower": 0.0,
        "upper": 10.0,
        "description": "Legacy translational drag scalar when non-ellipsoid hydro terms are active.",
    },
    {
        "key": "angular_drag",
        "label": "Legacy angular drag",
        "default": 0.72,
        "lower": 0.0,
        "upper": 10.0,
        "description": "Legacy angular drag scalar when non-ellipsoid hydro terms are active.",
    },
    {
        "key": "ellipsoid_model.effective_cd_linear",
        "label": "CD linear x y z",
        "kind": "vector3",
        "default": [0.06, 0.057, 0.087],
        "lower": 0.0,
        "upper": 3.0,
        "description": "Ellipsoid translational quadratic drag for surge, sway, heave.",
    },
    {
        "key": "ellipsoid_model.effective_cd_angular",
        "label": "CD angular r p y",
        "kind": "vector3",
        "default": [1.62, 0.78, 0.003],
        "lower": 0.0,
        "upper": 10.0,
        "description": "Ellipsoid rotational drag for roll, pitch, yaw.",
    },
    {
        "key": "mujoco_fluidcoef_scale",
        "label": "MuJoCo fluidcoef scale",
        "kind": "vector5",
        "default": [1.0, 1.0, 1.0, 1.0, 1.0],
        "lower": 0.0,
        "upper": 10.0,
        "description": "Active current-mode geom fluidcoef scale: blunt, slender, angular, Kutta, Magnus.",
    },
    {
        "key": "linear_damping_linear",
        "label": "6DOF linear damping xyz",
        "kind": "vector3",
        "default": [1.10, 1.32, 1.54],
        "lower": 0.0,
        "upper": 200.0,
        "description": "Custom/legacy 6-DOF translational linear damping. Used when fluid model is legacy.",
    },
    {
        "key": "linear_damping_angular",
        "label": "6DOF linear damping rpy",
        "kind": "vector3",
        "default": [0.32, 0.36, 0.28],
        "lower": 0.0,
        "upper": 200.0,
        "description": "Custom/legacy 6-DOF rotational linear damping. Used when fluid model is legacy.",
    },
    {
        "key": "quadratic_damping_linear",
        "label": "6DOF quad damping xyz",
        "kind": "vector3",
        "default": [1.40, 2.00, 2.40],
        "lower": 0.0,
        "upper": 500.0,
        "description": "Custom/legacy 6-DOF translational |v|v damping. Used when fluid model is legacy.",
    },
    {
        "key": "quadratic_damping_angular",
        "label": "6DOF quad damping rpy",
        "kind": "vector3",
        "default": [0.10, 0.12, 0.08],
        "lower": 0.0,
        "upper": 500.0,
        "description": "Custom/legacy 6-DOF rotational |omega|omega damping. Used when fluid model is legacy.",
    },
    {
        "key": "ellipsoid_model.added_mass_scale_linear",
        "label": "Added mass x y z",
        "kind": "vector3",
        "default": [0.45, 0.28, 0.65],
        "lower": 0.0,
        "upper": 5.0,
        "description": "Translational added-mass scale. Higher slows acceleration response.",
    },
    {
        "key": "ellipsoid_model.added_mass_scale_angular",
        "label": "Added mass r p y",
        "kind": "vector3",
        "default": [1.0, 0.92, 1.02],
        "lower": 0.0,
        "upper": 5.0,
        "description": "Rotational added-mass scale. Higher slows angular acceleration.",
    },
    {
        "key": "ellipsoid_model.linear_damping_ratio_linear",
        "label": "Low-speed linear damping",
        "default": 2.0,
        "lower": 0.0,
        "upper": 10.0,
        "description": "Low-speed translational damping blended with quadratic drag.",
    },
    {
        "key": "ellipsoid_model.linear_damping_ratio_angular",
        "label": "Low-speed angular damping",
        "default": 4.2,
        "lower": 0.0,
        "upper": 20.0,
        "description": "Low-speed angular damping for roll, pitch, yaw settling.",
    },
    {
        "key": "ellipsoid_model.reference_speed_linear",
        "label": "Ref speed linear",
        "default": 0.3,
        "lower": 0.01,
        "upper": 5.0,
        "description": "Reference speed used to derive ellipsoid low-speed linear damping.",
    },
    {
        "key": "ellipsoid_model.reference_speed_angular",
        "label": "Ref speed angular",
        "default": 0.6,
        "lower": 0.01,
        "upper": 10.0,
        "description": "Reference angular speed used to derive ellipsoid low-speed angular damping.",
    },
    {
        "key": "air_linear_drag",
        "label": "Air linear drag",
        "default": 0.03,
        "lower": 0.0,
        "upper": 10.0,
        "description": "Above-water translational damping fallback.",
    },
    {
        "key": "air_angular_drag",
        "label": "Air angular drag",
        "default": 0.03,
        "lower": 0.0,
        "upper": 10.0,
        "description": "Above-water rotational damping fallback.",
    },
    {
        "key": "current_world",
        "label": "Current world xyz",
        "kind": "vector3",
        "default": [0.0, 0.0, 0.0],
        "lower": -5.0,
        "upper": 5.0,
        "description": "Water-current velocity in world frame.",
    },
    {
        "key": "body_inertia_scale_xyz",
        "label": "Body inertia x y z",
        "kind": "vector3",
        "default": [1.0, 1.0, 1.0],
        "lower": 0.1,
        "upper": 5.0,
        "description": "Body rotational inertia scale. Affects roll/pitch/yaw acceleration.",
    },
    {
        "key": "yaw_torque_scale",
        "label": "Yaw torque scale",
        "default": 1.0,
        "lower": 0.0,
        "upper": 3.0,
        "description": "Additional yaw torque multiplier. Keep near 1 unless yaw rate is off.",
    },
)


@dataclass(frozen=True)
class RcLayout:
    label: str
    axis_channels: dict[str, int]
    summary: str


RC_LAYOUTS = {
    BACKEND_NONE: RcLayout(
        label="no ROS bridge detected",
        axis_channels={
            "heave": 2,    # ch3
            "yaw": 3,      # ch4
            "forward": 4,  # ch5
            "lateral": 5,  # ch6
        },
        summary="control unavailable until --ros2 or external MAVROS is running",
    ),
    BACKEND_MAVROS: RcLayout(
        label="legacy ArduSub/MAVROS",
        axis_channels={
            "heave": 2,    # ch3
            "yaw": 3,      # ch4
            "forward": 4,  # ch5
            "lateral": 5,  # ch6
        },
        summary="ch3=heave, ch4=yaw, ch5=forward, ch6=lateral",
    ),
    BACKEND_SIM_BRIDGE: RcLayout(
        label="MuJoCo sim bridge",
        axis_channels={
            "heave": 2,    # ch3
            "yaw": 3,      # ch4
            "forward": 4,  # ch5
            "lateral": 5,  # ch6
        },
        summary="ch3=heave, ch4=yaw, ch5=forward, ch6=lateral",
    ),
}

CMDVEL_AXIS_SCALE = {
    "forward": 0.40,
    "lateral": 0.35,
    "heave": 0.25,
    "yaw": 0.45,
}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp_axis(value: float) -> float:
    return clamp(float(value), AXIS_MIN, AXIS_MAX)


def normalize_axes(
    *,
    forward: float,
    lateral: float,
    heave: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    return (
        clamp_axis(forward),
        clamp_axis(lateral),
        clamp_axis(heave),
        clamp_axis(yaw),
    )


def format_age(age_s: float) -> str:
    if not math.isfinite(age_s):
        return "n/a"
    if age_s < 1.0:
        return f"{age_s * 1000.0:.0f} ms"
    return f"{age_s:.1f} s"


def format_replay_time(time_s: float) -> str:
    if not math.isfinite(time_s):
        return "--:--"
    time_s = max(0.0, float(time_s))
    minutes = int(time_s // 60.0)
    seconds = time_s - minutes * 60.0
    return f"{minutes:02d}:{seconds:04.1f}"


def severity_name(level: int) -> str:
    names = {
        StatusText.EMERGENCY: "EMERGENCY",
        StatusText.ALERT: "ALERT",
        StatusText.CRITICAL: "CRITICAL",
        StatusText.ERROR: "ERROR",
        StatusText.WARNING: "WARNING",
        StatusText.NOTICE: "NOTICE",
        StatusText.INFO: "INFO",
        StatusText.DEBUG: "DEBUG",
    }
    return names.get(level, f"S{level}")


def quaternion_to_euler_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Return roll, pitch, yaw in degrees."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def axis_to_pwm(value: float) -> int:
    """Map a normalized joystick axis in [-1, 1] to ArduSub RC PWM."""
    return int(round(RC_NEUTRAL_PWM + clamp_axis(value) * RC_PWM_SPAN))


def gui_rc_to_override_axes(
    *,
    forward: float,
    lateral: float,
    heave: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """Map GUI stick axes directly to RC override axes."""
    return normalize_axes(
        forward=forward,
        lateral=lateral,
        heave=heave,
        yaw=yaw,
    )


def make_rc_override_message(
    layout: RcLayout,
    *,
    yaw: float,
    heave: float,
    forward: float,
    lateral: float,
) -> OverrideRCIn:
    msg = OverrideRCIn()
    msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * RC_MESSAGE_CHANNEL_COUNT
    for idx in range(PRIMARY_RC_CHANNEL_COUNT):
        msg.channels[idx] = RC_NEUTRAL_PWM
    axes = {
        "heave": heave,
        "yaw": yaw,
        "forward": forward,
        "lateral": lateral,
    }
    for axis_name, axis_value in axes.items():
        msg.channels[int(layout.axis_channels[axis_name])] = axis_to_pwm(axis_value)
    return msg


def make_rc_release_message() -> OverrideRCIn:
    msg = OverrideRCIn()
    msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * RC_MESSAGE_CHANNEL_COUNT
    for idx in range(PRIMARY_RC_CHANNEL_COUNT):
        msg.channels[idx] = OverrideRCIn.CHAN_RELEASE
    return msg


def padded_rc_channels(
    values: Iterable[int],
    *,
    target_count: int = RC_FEEDBACK_CHANNEL_COUNT,
    sanitize_override_markers: bool = False,
) -> list[int]:
    channels: list[int] = []
    for value in list(values)[:target_count]:
        ivalue = int(value)
        if sanitize_override_markers and ivalue in (OverrideRCIn.CHAN_NOCHANGE, OverrideRCIn.CHAN_RELEASE):
            ivalue = 0
        elif sanitize_override_markers and (ivalue < 800 or ivalue > 2200):
            ivalue = 0
        channels.append(ivalue)
    if len(channels) < target_count:
        channels.extend([0] * (target_count - len(channels)))
    return channels


def rc_replay_bag_uri(path_text: str) -> str:
    path = Path(path_text).expanduser()
    if path.suffix == ".db3":
        return str(path.parent)
    return str(path)


def load_rc_override_replay(path_text: str) -> list[RcReplaySample]:
    if not HAVE_MAVROS_MSGS:
        raise RuntimeError("mavros_msgs is not available in this Python environment")
    if not HAVE_ROSBAG2_PY or rosbag2_py is None or deserialize_message is None:
        raise RuntimeError("rosbag2_py is not available in this Python environment")

    bag_uri = rc_replay_bag_uri(path_text)
    if not Path(bag_uri).exists():
        raise RuntimeError(f"bag path does not exist: {bag_uri}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if RC_REPLAY_TOPIC not in topics:
        raise RuntimeError(f"{RC_REPLAY_TOPIC} not found in bag")
    if topics[RC_REPLAY_TOPIC] != "mavros_msgs/msg/OverrideRCIn":
        raise RuntimeError(f"{RC_REPLAY_TOPIC} has unexpected type: {topics[RC_REPLAY_TOPIC]}")

    samples: list[RcReplaySample] = []
    first_timestamp_ns: int | None = None
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != RC_REPLAY_TOPIC:
            continue
        if first_timestamp_ns is None:
            first_timestamp_ns = int(timestamp_ns)
        msg = deserialize_message(data, OverrideRCIn)
        channels = padded_rc_channels(
            getattr(msg, "channels", []),
            target_count=RC_MESSAGE_CHANNEL_COUNT,
            sanitize_override_markers=False,
        )
        samples.append(
            RcReplaySample(
                time_s=(int(timestamp_ns) - first_timestamp_ns) * 1e-9,
                channels=tuple(channels[:RC_MESSAGE_CHANNEL_COUNT]),
            )
        )

    if not samples:
        raise RuntimeError(f"{RC_REPLAY_TOPIC} has no messages")
    return samples


def normalize_backend_name(name: str) -> str:
    value = str(name or BACKEND_AUTO).strip().lower()
    if value in ("none", "off", BACKEND_NONE):
        return BACKEND_NONE
    if value in ("sim", BACKEND_SIM_BRIDGE):
        return BACKEND_SIM_BRIDGE
    if value == BACKEND_MAVROS:
        return BACKEND_MAVROS
    return BACKEND_AUTO


def resolve_autotune_python() -> str:
    """Prefer the MuJoCo venv for offline replay/autotune helpers."""
    candidates: list[Path] = []
    env_python = os.environ.get("MJ311_PYTHON", "").strip()
    if env_python:
        candidates.append(Path(env_python).expanduser())
    env_root = os.environ.get("MJ311_ROOT", "").strip()
    if env_root:
        candidates.append(Path(env_root).expanduser() / "bin" / "python")
    candidates.append(Path(sys.executable))

    for candidate in candidates:
        if candidate.exists() and os.access(candidate, os.X_OK):
            return str(candidate)
    return sys.executable


@dataclass
class TelemetrySnapshot:
    connected: bool = False
    armed: bool = False
    guided: bool = False
    manual_input: bool = False
    mode: str = "UNKNOWN"
    mode_id: int = -1
    vehicle_mode: str = ""
    autopilot_name: str = ""
    system_status: int = 0

    battery_voltage: float = math.nan
    battery_current: float = math.nan
    battery_percent: float = math.nan

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    ang_vel_xyz: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    lin_acc_xyz: tuple[float, float, float] = (math.nan, math.nan, math.nan)

    position_xyz: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    velocity_xyz: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    velocity_source: str = "unavailable"

    depth_m: float = math.nan
    depth_source: str = "unavailable"
    pressure_pa: float = math.nan

    rc_out: list[int] = field(default_factory=lambda: [0] * RC_FEEDBACK_CHANNEL_COUNT)
    rc_feedback_source: str = "unavailable"
    events: Deque[str] = field(default_factory=lambda: deque(maxlen=TELEMETRY_EVENT_LIMIT))
    ping360_summary: str = "ping360: no status"
    ping360_age_s: float = math.inf

    state_age_s: float = math.inf
    imu_age_s: float = math.inf
    pose_age_s: float = math.inf
    depth_age_s: float = math.inf
    rc_age_s: float = math.inf


@dataclass(frozen=True)
class ControlCommands:
    velocity_forward: float
    velocity_lateral: float
    velocity_heave: float
    velocity_yaw: float
    rc_forward: float
    rc_lateral: float
    rc_heave: float
    rc_yaw: float


@dataclass(frozen=True)
class RcReplaySample:
    time_s: float
    channels: tuple[int, ...]


class UuvGuiNode(Node):
    def __init__(self, namespace: str, backend: str):
        super().__init__("uuv_control_gui")
        ns = namespace.rstrip("/")
        self._base_ns = ns if ns else ""
        self._backend_preference = normalize_backend_name(backend)
        self._backend_detected = (
            self._backend_preference
            if self._backend_preference != BACKEND_AUTO
            else DEFAULT_AUTO_BACKEND
        )

        self._lock = threading.Lock()
        self._snapshot = TelemetrySnapshot()
        self._last_wall = {}
        self._mode_request_in_flight = False
        self._vehicle_info_in_flight = False
        self._vehicle_info_supported = False
        self._last_graph_probe_wall = -1.0
        self._last_mode_seen = ""
        self._last_armed_seen: Optional[bool] = None
        self._rc_override_subscribers = 0

        if HAVE_MAVROS_MSGS:
            self._rc_override_pub = self.create_publisher(OverrideRCIn, self._topic("rc/override"), 10)
            self._arm_client = self.create_client(CommandBool, self._topic("cmd/arming"))
            self._mode_client = self.create_client(SetMode, self._topic("set_mode"))
            self._vehicle_info_client = self.create_client(VehicleInfoGet, self._topic("vehicle_info_get"))
        else:
            self._rc_override_pub = None
            self._arm_client = None
            self._mode_client = None
            self._vehicle_info_client = None

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self._ping360_config_pub = self.create_publisher(String, "/ping360/config", 10)
        self.create_subscription(String, "/ping360/status", self._on_ping360_status, 10)

        self.create_subscription(Imu, self._topic("imu/data"), self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(BatteryState, self._topic("battery"), self._on_battery, best_effort_qos)
        self.create_subscription(BatteryState, "/battery", self._on_battery, best_effort_qos)
        if HAVE_MAVROS_MSGS:
            self.create_subscription(State, self._topic("state"), self._on_state, state_qos)
            self.create_subscription(
                PoseStamped, self._topic("local_position/pose"), self._on_pose, qos_profile_sensor_data
            )
            self.create_subscription(
                Odometry, self._topic("local_position/odom"), self._on_local_odom, qos_profile_sensor_data
            )
            self.create_subscription(
                TwistStamped,
                self._topic("local_position/velocity_body"),
                self._on_velocity_body,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                TwistStamped,
                self._topic("local_position/velocity_local"),
                self._on_velocity_local,
                qos_profile_sensor_data,
            )
        self.create_subscription(Odometry, "/rovio/odometry", self._on_rovio_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/dvl/odometry", self._on_dvl_odom, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/dvl/velocity", self._on_dvl_velocity, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/mujoco/ground_truth/pose", self._on_ground_truth_pose, qos_profile_sensor_data)
        if HAVE_MAVROS_MSGS:
            self.create_subscription(RCOut, self._topic("rc/out"), self._on_rc_out, 20)
            self.create_subscription(OverrideRCIn, self._topic("rc/in"), self._on_rc_in, 20)
            self.create_subscription(StatusText, self._topic("statustext/recv"), self._on_status_text, best_effort_qos)

        self.create_subscription(Float32, "/depth", self._on_depth, best_effort_qos)
        self.create_subscription(Float32, "/bar30/pressure_pa", self._on_bar30_pressure, best_effort_qos)
        if HAVE_MAVROS_MSGS:
            self.create_subscription(
                FluidPressure,
                self._topic("imu/atm_pressure"),
                self._on_atm_pressure,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                FluidPressure,
                self._topic("imu/static_pressure"),
                self._on_static_pressure,
                qos_profile_sensor_data,
            )

        self._push_event(
            f"GUI attached to {self._base_ns or '/mavros'} "
            f"(RC override via {self._topic('rc/override')}; SITL stack uses ArduSub closed-loop)"
        )
        if not HAVE_MAVROS_MSGS:
            self._push_event("mavros_msgs not available in this Python env: MAVROS arm/mode/RC features disabled")
        self._probe_backend(force=True)

    def _topic(self, suffix: str) -> str:
        if not self._base_ns:
            return f"/{suffix.lstrip('/')}"
        return f"{self._base_ns}/{suffix.lstrip('/')}"

    def _safe_count_publishers(self, topic: str) -> int:
        try:
            return int(self.count_publishers(topic))
        except Exception:
            return 0

    def _safe_count_subscribers(self, topic: str) -> int:
        try:
            return int(self.count_subscribers(topic))
        except Exception:
            return 0

    @staticmethod
    def _service_ready(client) -> int:
        if client is None:
            return 0
        try:
            return 1 if client.service_is_ready() else 0
        except Exception:
            return 0

    def _effective_backend(self) -> str:
        if self._backend_preference != BACKEND_AUTO:
            return self._backend_preference
        return self._backend_detected

    def _active_layout(self) -> RcLayout:
        backend = self._effective_backend()
        return RC_LAYOUTS.get(backend, RC_LAYOUTS[DEFAULT_AUTO_BACKEND])

    def backend_label(self) -> str:
        backend = self._effective_backend()
        layout = self._active_layout()
        if self._backend_preference == BACKEND_AUTO:
            return f"auto->{backend} ({layout.label})"
        return f"{backend} ({layout.label})"

    def rc_mapping_summary(self) -> str:
        return self._active_layout().summary

    def vehicle_info_supported(self) -> bool:
        return bool(self._vehicle_info_supported)

    def probe_backend(self) -> None:
        self._probe_backend()

    def _probe_backend(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and self._last_graph_probe_wall >= 0.0 and (now - self._last_graph_probe_wall) < 1.0:
            return
        self._last_graph_probe_wall = now

        mavros_score = 0
        sim_score = 0

        vehicle_info_services = self._service_ready(self._vehicle_info_client)
        arm_services = self._service_ready(self._arm_client)
        mode_services = self._service_ready(self._mode_client)
        rc_out_publishers = self._safe_count_publishers(self._topic("rc/out"))
        state_publishers = self._safe_count_publishers(self._topic("state"))
        pose_publishers = self._safe_count_publishers(self._topic("local_position/pose"))
        velocity_body_publishers = self._safe_count_publishers(self._topic("local_position/velocity_body"))
        rc_in_publishers = self._safe_count_publishers(self._topic("rc/in"))
        velocity_local_publishers = self._safe_count_publishers(
            self._topic("local_position/velocity_local")
        )
        bridge_imu_publishers = self._safe_count_publishers("/imu/data")
        bridge_battery_publishers = self._safe_count_publishers("/battery")
        bridge_rovio_publishers = self._safe_count_publishers("/rovio/odometry")
        bridge_dvl_odom_publishers = self._safe_count_publishers("/dvl/odometry")
        bridge_dvl_velocity_publishers = self._safe_count_publishers("/dvl/velocity")
        bridge_depth_publishers = self._safe_count_publishers("/depth")
        rc_override_subscribers = self._safe_count_subscribers(self._topic("rc/override"))
        self._rc_override_subscribers = rc_override_subscribers

        if vehicle_info_services > 0:
            mavros_score += 3
        if arm_services > 0:
            mavros_score += 1
        if mode_services > 0:
            mavros_score += 1
        if rc_out_publishers > 0:
            mavros_score += 3
        if state_publishers > 0:
            mavros_score += 1
        if pose_publishers > 0:
            mavros_score += 1
        if velocity_body_publishers > 0:
            mavros_score += 3
        if rc_in_publishers > 0:
            sim_score += 3
        if velocity_local_publishers > 0:
            sim_score += 2
        if bridge_imu_publishers > 0:
            sim_score += 2
        if bridge_rovio_publishers > 0:
            sim_score += 3
        if bridge_dvl_odom_publishers > 0:
            sim_score += 2
        if bridge_dvl_velocity_publishers > 0:
            sim_score += 1
        if bridge_depth_publishers > 0:
            sim_score += 1
        if bridge_battery_publishers > 0:
            sim_score += 1
        if rc_override_subscribers > 0:
            sim_score += 2

        self._vehicle_info_supported = vehicle_info_services > 0
        if self._backend_preference != BACKEND_AUTO:
            return

        next_backend = self._backend_detected
        if mavros_score > 0 and (
            mavros_score > sim_score
            or (
                mavros_score == sim_score
                and (rc_out_publishers > 0 or velocity_body_publishers > 0 or vehicle_info_services > 0)
            )
        ):
            next_backend = BACKEND_MAVROS
        elif sim_score > 0:
            next_backend = BACKEND_SIM_BRIDGE
        else:
            next_backend = DEFAULT_AUTO_BACKEND

        if next_backend != self._backend_detected:
            self._backend_detected = next_backend
            self._push_event(
                f"backend -> {self.backend_label()} ({self.rc_mapping_summary()})"
            )

    def _touch(self, key: str) -> None:
        self._last_wall[key] = time.monotonic()

    def _push_event(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        with self._lock:
            self._snapshot.events.appendleft(f"[{stamp}] {text}")

    def push_event(self, text: str) -> None:
        self._push_event(text)

    def snapshot(self) -> TelemetrySnapshot:
        now = time.monotonic()
        with self._lock:
            snap = replace(
                self._snapshot,
                rc_out=list(self._snapshot.rc_out),
                events=deque(self._snapshot.events, maxlen=TELEMETRY_EVENT_LIMIT),
            )

        snap.state_age_s = now - self._last_wall.get("state", math.inf)
        snap.imu_age_s = now - self._last_wall.get("imu", math.inf)
        snap.pose_age_s = now - self._last_wall.get("pose", math.inf)
        snap.depth_age_s = now - self._last_wall.get("depth", math.inf)
        snap.rc_age_s = now - self._last_wall.get("rc_out", math.inf)
        snap.ping360_age_s = now - self._last_wall.get("ping360", math.inf)
        return snap

    def request_vehicle_info(self) -> None:
        self._probe_backend()
        if not self._vehicle_info_supported:
            return
        if self._vehicle_info_in_flight:
            return
        try:
            ready = self._vehicle_info_client.service_is_ready()
        except Exception:
            return
        if not ready:
            return
        req = VehicleInfoGet.Request()
        req.sysid = 1
        req.compid = 1
        req.get_all = False
        future = self._vehicle_info_client.call_async(req)
        self._vehicle_info_in_flight = True
        future.add_done_callback(self._on_vehicle_info_response)

    def _on_vehicle_info_response(self, future) -> None:
        self._vehicle_info_in_flight = False
        try:
            resp = future.result()
        except Exception as exc:
            self._push_event(f"vehicle_info_get failed: {exc}")
            return
        if not resp.success or not resp.vehicles:
            return
        info = resp.vehicles[0]
        with self._lock:
            self._snapshot.vehicle_mode = info.mode
            self._snapshot.mode_id = int(info.mode_id)
            self._snapshot.autopilot_name = f"autopilot={info.autopilot}, type={info.type}"

    def arm(self, value: bool) -> None:
        if self._arm_client is None:
            self._push_event("arm service unavailable in current Python env")
            return
        try:
            ready = self._arm_client.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            self._push_event("arm service unavailable")
            return
        req = CommandBool.Request()
        req.value = bool(value)
        future = self._arm_client.call_async(req)
        future.add_done_callback(
            lambda fut: self._on_arm_response(fut, "arm" if value else "disarm")
        )

    def _on_arm_response(self, future, action: str) -> None:
        try:
            resp = future.result()
        except Exception as exc:
            self._push_event(f"{action} failed: {exc}")
            return
        self._push_event(f"{action}: success={resp.success}, result={resp.result}")

    def set_mode(self, mode: str) -> None:
        if self._mode_request_in_flight:
            return
        if self._mode_client is None:
            self._push_event("set_mode service unavailable in current Python env")
            return
        try:
            ready = self._mode_client.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            self._push_event("set_mode service unavailable")
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        self._mode_request_in_flight = True
        future.add_done_callback(lambda fut: self._on_mode_response(fut, mode))

    def _on_mode_response(self, future, mode: str) -> None:
        self._mode_request_in_flight = False
        try:
            resp = future.result()
        except Exception as exc:
            self._push_event(f"set_mode {mode} failed: {exc}")
            return
        self._push_event(f"set_mode {mode}: mode_sent={resp.mode_sent}")

    def publish_rc_override(
        self,
        *,
        yaw: float,
        heave: float,
        forward: float,
        lateral: float,
        pitch: float = 0.0,
        roll: float = 0.0,
    ) -> None:
        del pitch
        del roll
        if self._rc_override_pub is None:
            return
        self._probe_backend(force=True)
        msg = make_rc_override_message(
            self._active_layout(),
            yaw=yaw,
            heave=heave,
            forward=forward,
            lateral=lateral,
        )
        if self._rc_override_subscribers > 0:
            self._rc_override_pub.publish(msg)

    def publish_rc_release(self) -> None:
        if self._rc_override_pub is None:
            return
        msg = make_rc_release_message()
        self._rc_override_pub.publish(msg)

    def publish_rc_channels(self, channels: Iterable[int]) -> bool:
        if self._rc_override_pub is None:
            return False
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * RC_MESSAGE_CHANNEL_COUNT
        for idx, value in enumerate(list(channels)[:RC_MESSAGE_CHANNEL_COUNT]):
            msg.channels[idx] = int(value)
        self._rc_override_pub.publish(msg)
        return True

    def publish_ping360_config(
        self,
        *,
        range_m: float,
        num_steps: int,
        gain: int,
        interface_mode: str,
        frequency_khz: int,
        start_angle_grad: int,
        stop_angle_grad: int,
    ) -> None:
        payload = {
            "requested_range_m": float(range_m),
            "num_steps": int(num_steps),
            "gain_setting": int(gain),
            "interface_mode": str(interface_mode),
            "transmit_frequency_khz": int(frequency_khz),
            "start_angle_grad": int(start_angle_grad),
            "stop_angle_grad": int(stop_angle_grad),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._ping360_config_pub.publish(msg)
        self._push_event(
            "ping360 config -> "
            f"range={range_m:g}m step={num_steps} gain={gain} "
            f"{interface_mode} {frequency_khz}kHz sector={start_angle_grad}..{stop_angle_grad}grad"
        )

    def _on_state(self, msg: State) -> None:
        self._touch("state")
        with self._lock:
            self._snapshot.connected = bool(msg.connected)
            self._snapshot.armed = bool(msg.armed)
            self._snapshot.guided = bool(msg.guided)
            self._snapshot.manual_input = bool(msg.manual_input)
            self._snapshot.mode = msg.mode
            self._snapshot.system_status = int(msg.system_status)

        if msg.mode != self._last_mode_seen:
            self._push_event(f"mode -> {msg.mode}")
            self._last_mode_seen = msg.mode
        if self._last_armed_seen is None or bool(msg.armed) != self._last_armed_seen:
            self._push_event(f"armed -> {msg.armed}")
            self._last_armed_seen = bool(msg.armed)

    def _on_imu(self, msg: Imu) -> None:
        self._touch("imu")
        q = msg.orientation
        roll_deg, pitch_deg, yaw_deg = quaternion_to_euler_deg(q.w, q.x, q.y, q.z)
        with self._lock:
            self._snapshot.roll_deg = roll_deg
            self._snapshot.pitch_deg = pitch_deg
            self._snapshot.yaw_deg = yaw_deg
            self._snapshot.ang_vel_xyz = (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            )
            self._snapshot.lin_acc_xyz = (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._snapshot.battery_voltage = msg.voltage
            self._snapshot.battery_current = msg.current
            self._snapshot.battery_percent = msg.percentage

    def _on_pose(self, msg: PoseStamped) -> None:
        self._touch("pose")
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        with self._lock:
            self._snapshot.position_xyz = (x, y, z)
            depth_age = time.monotonic() - self._last_wall.get("depth", math.inf)
            if not math.isfinite(self._snapshot.depth_m) or depth_age > 1.0:
                self._snapshot.depth_m = max(0.0, -z)
                self._snapshot.depth_source = "local_position.pose.z"

    def _on_odom(self, msg: Odometry, source: str) -> None:
        self._touch("pose")
        pose = msg.pose.pose
        twist = msg.twist.twist
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        with self._lock:
            self._snapshot.position_xyz = (x, y, z)
            self._snapshot.velocity_xyz = (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
            )
            self._snapshot.velocity_source = source
            depth_age = time.monotonic() - self._last_wall.get("depth", math.inf)
            if not math.isfinite(self._snapshot.depth_m) or depth_age > 1.0:
                self._snapshot.depth_m = max(0.0, -z)
                self._snapshot.depth_source = f"{source}.pose.z"

    def _on_local_odom(self, msg: Odometry) -> None:
        self._on_odom(msg, self._topic("local_position/odom"))

    def _on_rovio_odom(self, msg: Odometry) -> None:
        self._on_odom(msg, "/rovio/odometry")

    def _on_dvl_odom(self, msg: Odometry) -> None:
        self._on_odom(msg, "/dvl/odometry")

    def _on_velocity(self, msg: TwistStamped, source: str) -> None:
        with self._lock:
            self._snapshot.velocity_xyz = (
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
            )
            self._snapshot.velocity_source = source

    def _on_velocity_body(self, msg: TwistStamped) -> None:
        self._on_velocity(msg, self._topic("local_position/velocity_body"))

    def _on_velocity_local(self, msg: TwistStamped) -> None:
        self._on_velocity(msg, self._topic("local_position/velocity_local"))

    def _on_dvl_velocity(self, msg: TwistStamped) -> None:
        self._on_velocity(msg, "/dvl/velocity")

    def _on_ground_truth_pose(self, msg: PoseStamped) -> None:
        self._touch("pose")
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        with self._lock:
            if not math.isfinite(self._snapshot.position_xyz[0]):
                self._snapshot.position_xyz = (x, y, z)
            depth_age = time.monotonic() - self._last_wall.get("depth", math.inf)
            if not math.isfinite(self._snapshot.depth_m) or depth_age > 1.0:
                self._snapshot.depth_m = max(0.0, -z)
                self._snapshot.depth_source = "/mujoco/ground_truth/pose.z"

    def _on_rc_out(self, msg: RCOut) -> None:
        self._touch("rc_out")
        with self._lock:
            self._snapshot.rc_out = padded_rc_channels(msg.channels)
            self._snapshot.rc_feedback_source = self._topic("rc/out")

    def _on_rc_in(self, msg: OverrideRCIn) -> None:
        self._touch("rc_out")
        with self._lock:
            self._snapshot.rc_out = padded_rc_channels(
                getattr(msg, "channels", []),
                sanitize_override_markers=True,
            )
            self._snapshot.rc_feedback_source = self._topic("rc/in")

    def _on_status_text(self, msg: StatusText) -> None:
        self._push_event(f"{severity_name(int(msg.severity))}: {msg.text}")

    def _on_depth(self, msg: Float32) -> None:
        self._touch("depth")
        with self._lock:
            self._snapshot.depth_m = float(msg.data)
            self._snapshot.depth_source = "/depth"

    def _on_bar30_pressure(self, msg: Float32) -> None:
        self._on_pressure_value(float(msg.data), "/bar30/pressure_pa")

    def _on_ping360_status(self, msg: String) -> None:
        self._touch("ping360")
        try:
            payload = json.loads(str(msg.data))
            settings = payload.get("settings", {}) if isinstance(payload, dict) else {}
        except json.JSONDecodeError:
            settings = {}
            payload = {}

        try:
            effective_range = float(settings.get("effective_range_m", math.nan))
            requested_range = float(settings.get("requested_range_m", math.nan))
            resolution_cm = float(settings.get("range_resolution_m", math.nan)) * 100.0
            angular_resolution = float(settings.get("angular_resolution_deg", math.nan))
            scan_period = float(settings.get("scan_period_s", math.nan))
            angle_deg = float(payload.get("angle_deg", math.nan))
            num_steps = int(settings.get("num_steps", 0))
            start_grad = int(settings.get("start_angle_grad", 0))
            stop_grad = int(settings.get("stop_angle_grad", 399))
            flags = settings.get("quality_flags", [])
        except (TypeError, ValueError):
            effective_range = requested_range = resolution_cm = angular_resolution = scan_period = angle_deg = math.nan
            num_steps = 0
            start_grad = 0
            stop_grad = 399
            flags = []

        if math.isfinite(effective_range):
            summary = (
                f"ping360: req={requested_range:.2f}m eff={effective_range:.2f}m "
                f"res={resolution_cm:.2f}cm step={num_steps}/{angular_resolution:.1f}deg "
                f"sector={start_grad}..{stop_grad}grad scan={scan_period:.1f}s "
                f"angle={angle_deg:.1f}deg"
            )
        else:
            summary = "ping360: waiting for status"
        if isinstance(flags, list) and flags:
            summary += " flags=" + ",".join(str(flag) for flag in flags[:4])
        with self._lock:
            self._snapshot.ping360_summary = summary

    def _on_atm_pressure(self, msg: FluidPressure) -> None:
        self._on_pressure_value(float(msg.fluid_pressure), self._topic("imu/atm_pressure"))

    def _on_static_pressure(self, msg: FluidPressure) -> None:
        self._on_pressure_value(float(msg.fluid_pressure), self._topic("imu/static_pressure"))

    def _on_pressure_value(self, pressure_pa: float, source: str) -> None:
        self._touch("depth")
        rho = 997.0
        g = 9.80665
        approx_depth = max(0.0, (pressure_pa - 101325.0) / (rho * g))
        with self._lock:
            self._snapshot.pressure_pa = pressure_pa
            if not math.isfinite(self._snapshot.depth_m):
                self._snapshot.depth_m = approx_depth
                self._snapshot.depth_source = f"{source} (approx)"


class VirtualJoystick(ttk.LabelFrame):
    def __init__(
        self,
        parent,
        *,
        title: str,
        x_var: tk.DoubleVar,
        y_var: tk.DoubleVar,
        x_label: str,
        y_label: str,
    ) -> None:
        super().__init__(parent, text=title, padding=INNER_PADDING)
        self.x_var = x_var
        self.y_var = y_var
        self.x_label = x_label
        self.y_label = y_label

        self.canvas = tk.Canvas(
            self,
            width=JOYSTICK_CANVAS_SIZE,
            height=JOYSTICK_CANVAS_SIZE,
            bg="#0b1220",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.value_label = ttk.Label(self, anchor="center")
        self.value_label.pack(fill=tk.X, pady=(3, 0))

        self.canvas.bind("<Configure>", self._redraw)
        self.canvas.bind("<Button-1>", self._on_drag)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Double-Button-1>", self._on_release)

        self.x_var.trace_add("write", self._redraw)
        self.y_var.trace_add("write", self._redraw)
        self._redraw()

    def _geometry(self) -> tuple[int, int, float, float, float]:
        width = max(self.canvas.winfo_width(), 130)
        height = max(self.canvas.winfo_height(), 130)
        cx = width / 2.0
        cy = height / 2.0
        radius = min(width, height) * 0.34
        return width, height, cx, cy, radius

    def _set_axes(self, x: float, y: float) -> None:
        if abs(x) < AXIS_DEADBAND:
            x = 0.0
        if abs(y) < AXIS_DEADBAND:
            y = 0.0
        self.x_var.set(clamp_axis(x))
        self.y_var.set(clamp_axis(y))

    def _on_drag(self, event) -> None:
        _, _, cx, cy, radius = self._geometry()
        x = (event.x - cx) / radius
        y = (cy - event.y) / radius
        self._set_axes(x, y)

    def _on_release(self, _event=None) -> None:
        self._set_axes(0.0, 0.0)

    def _redraw(self, *_args) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width, height, cx, cy, radius = self._geometry()
        x = clamp_axis(self.x_var.get())
        y = clamp_axis(self.y_var.get())

        canvas.create_rectangle(10, 10, width - 10, height - 10, outline="#334155", width=2)
        canvas.create_line(cx, 20, cx, height - 20, fill="#334155", width=2)
        canvas.create_line(20, cy, width - 20, cy, fill="#334155", width=2)
        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#475569", width=2)

        knob_x = cx + x * radius
        knob_y = cy - y * radius
        canvas.create_line(cx, cy, knob_x, knob_y, fill="#38bdf8", width=3)
        knob_radius = max(10, min(width, height) * 0.055)
        canvas.create_oval(
            knob_x - knob_radius,
            knob_y - knob_radius,
            knob_x + knob_radius,
            knob_y + knob_radius,
            fill="#0ea5e9",
            outline="#e0f2fe",
            width=2,
        )

        canvas.create_text(cx, 18, text=self.y_label, fill="#cbd5e1")
        canvas.create_text(width - 18, cy - 10, text=self.x_label, fill="#cbd5e1", anchor="e")
        canvas.create_text(18, cy - 10, text=f"-{self.x_label}", fill="#64748b", anchor="w")
        canvas.create_text(cx, height - 18, text=f"-{self.y_label}", fill="#64748b")

        self.value_label.config(text=f"{self.y_label}={y:+.2f}  {self.x_label}={x:+.2f}")


class UuvControlGui:
    MODE_BUTTONS = ("MANUAL", "STABILIZE", "ALT_HOLD", "GUIDED", "SURFACE", "POSHOLD")

    def __init__(self, node: UuvGuiNode, title: str):
        self.node = node
        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry(WINDOW_GEOMETRY)
        self.root.minsize(*WINDOW_MINSIZE)

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self.control_enabled = tk.BooleanVar(value=False)
        self.rc_override_enabled = tk.BooleanVar(value=False)
        self.control_details_visible = tk.BooleanVar(value=False)
        self.vehicle_details_visible = tk.BooleanVar(value=False)
        self.telemetry_visible = tk.BooleanVar(value=True)
        self.autotune_visible = tk.BooleanVar(value=False)
        self.forward_var = tk.DoubleVar(value=0.0)
        self.lateral_var = tk.DoubleVar(value=0.0)
        self.heave_var = tk.DoubleVar(value=0.0)
        self.yaw_var = tk.DoubleVar(value=0.0)
        self.rc_forward_var = tk.DoubleVar(value=0.0)
        self.rc_lateral_var = tk.DoubleVar(value=0.0)
        self.rc_heave_var = tk.DoubleVar(value=0.0)
        self.rc_yaw_var = tk.DoubleVar(value=0.0)
        self.vehicle_summary_var = tk.StringVar(value="vehicle: disconnected")
        self.motion_summary_var = tk.StringVar(value="motion: n/a")
        self.depth_target_var = tk.StringVar(value="n/a")
        self.depth_source_var = tk.StringVar(value="depth source: unavailable")
        self.mode_var = tk.StringVar(value="mode: UNKNOWN")
        self.status_var = tk.StringVar(value="disconnected")
        self.battery_var = tk.StringVar(value="battery: n/a")
        self.pose_var = tk.StringVar(value="pose: n/a")
        self.vel_var = tk.StringVar(value="velocity: n/a")
        self.imu_var = tk.StringVar(value="imu: n/a")
        self.autopilot_var = tk.StringVar(value="autopilot: n/a")
        self.age_var = tk.StringVar(value="state age: n/a")
        self.control_summary_var = tk.StringVar(value="control: idle")
        self.rc_override_var = tk.StringVar(value="rc override: off")
        self.control_var = tk.StringVar(value="setpoint: x=0.00 y=0.00 z=0.00 yaw=0.00")
        self.rc_replay_path_var = tk.StringVar(value=str(DEFAULT_RC_REPLAY_BAG))
        self.rc_replay_rate_var = tk.StringVar(value="1.0")
        self.rc_replay_status_var = tk.StringVar(value="replay: unloaded")
        self.rc_replay_position_var = tk.DoubleVar(value=0.0)
        self.rc_replay_time_var = tk.StringVar(value="00:00.0 / 00:00.0")
        self.physics_status_var = tk.StringVar(value="physics params: idle")
        self.physics_param_vars = {
            str(spec["key"]): tk.StringVar(value="")
            for spec in PHYSICS_PARAM_SPECS
        }
        self.autotune_bag_var = tk.StringVar(value=str(DEFAULT_AUTOTUNE_BAG))
        self.autotune_start_var = tk.StringVar(value="60")
        self.autotune_duration_var = tk.StringVar(value="120")
        self.autotune_candidates_var = tk.StringVar(value="15")
        self.autotune_servo_scale_var = tk.StringVar(value="0.58")
        self.autotune_mode_var = tk.StringVar(value="plant-rc-out")
        self.autotune_candidate_set_var = tk.StringVar(value="ellipsoid5")
        self.autotune_status_var = tk.StringVar(value="autotune: idle")
        self.autotune_out_dir_var = tk.StringVar(value="")
        self.autotune_apply_best_var = tk.BooleanVar(value=False)
        self.sim_stack_status_var = tk.StringVar(value="sim: stopped")
        self.ping360_view_status_var = tk.StringVar(value="ping360 view: closed")
        self.ping360_range_var = tk.StringVar(value="2.0")
        self.ping360_num_steps_var = tk.StringVar(value="1")
        self.ping360_gain_var = tk.StringVar(value="0")
        self.ping360_interface_var = tk.StringVar(value="ethernet")
        self.ping360_frequency_var = tk.StringVar(value="750")
        self.ping360_start_angle_var = tk.StringVar(value="0")
        self.ping360_stop_angle_var = tk.StringVar(value="399")
        self.ping360_summary_var = tk.StringVar(value="ping360: no status")

        self._last_event_top = ""
        self._last_vehicle_info_wall = 0.0
        self._guided_control_prev = False
        self._rc_override_prev = False
        self._rc_replay_samples: list[RcReplaySample] = []
        self._rc_replay_thread: Optional[threading.Thread] = None
        self._rc_replay_duration_s = 0.0
        self._rc_replay_slider_dragging = False
        self._rc_replay_seek_lock = threading.Lock()
        self._rc_replay_seek_time_s: Optional[float] = None
        self._rc_replay_stop_event = threading.Event()
        self._rc_replay_pause_event = threading.Event()
        self._autotune_process: subprocess.Popen[str] | None = None
        self._autotune_thread: threading.Thread | None = None
        self._sim_stack_process: subprocess.Popen[str] | None = None
        self._ping360_view_process: subprocess.Popen[str] | None = None
        self._ping360_view_log_path: Path | None = None
        self.ping360_window: tk.Toplevel | None = None
        self._sim_stack_thread: threading.Thread | None = None
        self._sim_stack_reset_thread: threading.Thread | None = None
        self._sim_stack_log_path: Path | None = None
        self.autotune_monitor_window: tk.Toplevel | None = None
        self.autotune_monitor_status_var = tk.StringVar(value="autotune monitor: idle")
        self.autotune_monitor_progress_var = tk.DoubleVar(value=0.0)
        self.autotune_tree: ttk.Treeview | None = None
        self.autotune_chart_canvas: tk.Canvas | None = None
        self.autotune_log_text: tk.Text | None = None
        self._autotune_candidate_order: list[str] = []
        self._autotune_candidate_rows: dict[str, dict[str, Any]] = {}
        self._autotune_current_candidate: str | None = None
        self._closed = False
        self._after_id: Optional[str] = None
        self.main_container: ttk.Frame | None = None
        self.telemetry_panel: ttk.Frame | None = None
        self.control_panel: ttk.Frame | None = None
        self.physics_window: tk.Toplevel | None = None
        self.physics_canvas: tk.Canvas | None = None
        self.physics_scroll_frame: ttk.Frame | None = None
        self.autotune_frame: ttk.LabelFrame | None = None

        self._build_layout()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._schedule_update()

    def _spin(self) -> None:
        self._executor.spin()

    def _schedule_update(self) -> None:
        if not self._closed and self.root.winfo_exists():
            self._after_id = self.root.after(UI_UPDATE_PERIOD_MS, self._update_ui)

    def _build_layout(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", font=("TkDefaultFont", 10))
        style.configure("TButton", padding=(5, 2))
        style.configure("Accent.TButton", padding=(5, 2), foreground="white", background="#2563eb")
        style.map("Accent.TButton", background=[("active", "#1d4ed8"), ("pressed", "#1e40af")])
        style.configure("Info.TButton", padding=(5, 2), foreground="white", background="#0ea5e9")
        style.map("Info.TButton", background=[("active", "#0284c7"), ("pressed", "#0369a1")])
        style.configure("TCheckbutton", padding=(2, 1))
        style.configure("TLabelframe", padding=2)

        container = ttk.Frame(self.root, padding=OUTER_PADDING)
        self.main_container = container
        container.pack(fill=tk.BOTH, expand=True)
        container.columnconfigure(0, weight=3)
        container.columnconfigure(1, weight=2)
        container.rowconfigure(0, weight=1)

        left = ttk.Frame(container)
        self.telemetry_panel = left
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)
        left.rowconfigure(2, weight=1)

        right = ttk.Frame(container)
        self.control_panel = right
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)

        summary = ttk.LabelFrame(left, text="Vehicle Summary", padding=GROUP_PADDING)
        summary.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        summary.columnconfigure(0, weight=1)

        summary_header = ttk.Frame(summary)
        summary_header.grid(row=0, column=0, sticky="ew")
        summary_header.columnconfigure(0, weight=1)
        ttk.Label(
            summary_header,
            textvariable=self.vehicle_summary_var,
            anchor="w",
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=0, column=0, sticky="ew")
        self.vehicle_details_button = ttk.Button(
            summary_header,
            text="Details >",
            width=10,
            command=self._toggle_vehicle_details,
        )
        self.vehicle_details_button.grid(row=0, column=1, sticky="e", padx=(6, 0))

        ttk.Label(summary, textvariable=self.motion_summary_var, anchor="w").grid(
            row=1, column=0, sticky="ew", pady=(2, 0)
        )
        ttk.Label(summary, textvariable=self.control_summary_var, anchor="w").grid(
            row=2, column=0, sticky="ew", pady=(1, 0)
        )

        self.vehicle_details_frame = ttk.Frame(summary)
        self.vehicle_details_frame.grid(row=3, column=0, sticky="ew", pady=(4, 0))
        self.vehicle_details_frame.columnconfigure(0, weight=1)
        for row, var in enumerate(
            (
                self.status_var,
                self.mode_var,
                self.battery_var,
                self.pose_var,
                self.vel_var,
                self.imu_var,
                self.autopilot_var,
                self.depth_target_var,
                self.depth_source_var,
                self.age_var,
            )
        ):
            ttk.Label(self.vehicle_details_frame, textvariable=var, anchor="w").grid(
                row=row, column=0, sticky="ew", pady=1
            )
        self.vehicle_details_frame.grid_remove()

        visuals = ttk.Frame(left)
        visuals.grid(row=1, column=0, sticky="nsew", pady=(0, 6))
        visuals.columnconfigure(0, weight=3)
        visuals.columnconfigure(1, weight=2)
        visuals.rowconfigure(0, weight=1)

        attitude_box = ttk.LabelFrame(visuals, text="Attitude", padding=GROUP_PADDING)
        attitude_box.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.attitude_canvas = tk.Canvas(
            attitude_box,
            width=ATTITUDE_CANVAS_WIDTH,
            height=ATTITUDE_CANVAS_HEIGHT,
            bg="#0f172a",
            highlightthickness=0,
        )
        self.attitude_canvas.pack(fill=tk.BOTH, expand=True)

        side_box = ttk.Frame(visuals)
        side_box.grid(row=0, column=1, sticky="nsew")
        side_box.rowconfigure(0, weight=1)
        side_box.rowconfigure(1, weight=1)
        side_box.columnconfigure(0, weight=1)

        depth_box = ttk.LabelFrame(side_box, text="Depth", padding=GROUP_PADDING)
        depth_box.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        self.depth_canvas = tk.Canvas(
            depth_box,
            width=DEPTH_CANVAS_WIDTH,
            height=DEPTH_CANVAS_HEIGHT,
            bg="#081018",
            highlightthickness=0,
        )
        self.depth_canvas.pack(fill=tk.BOTH, expand=True)

        rc_box = ttk.LabelFrame(side_box, text="RC Feedback", padding=GROUP_PADDING)
        rc_box.grid(row=1, column=0, sticky="nsew")
        self._rc_bars = []
        self._rc_labels = []
        for idx in range(RC_VISIBLE_CHANNEL_COUNT):
            ttk.Label(rc_box, text=f"Ch {idx + 1:02d}").grid(row=idx, column=0, sticky="w")
            bar = ttk.Progressbar(rc_box, orient=tk.HORIZONTAL, maximum=800, mode="determinate")
            bar.grid(row=idx, column=1, sticky="ew", padx=6)
            value_label = ttk.Label(rc_box, text="0")
            value_label.grid(row=idx, column=2, sticky="e")
            self._rc_bars.append(bar)
            self._rc_labels.append(value_label)
        rc_box.columnconfigure(1, weight=1)

        log_box = ttk.LabelFrame(left, text="Events", padding=GROUP_PADDING)
        log_box.grid(row=2, column=0, sticky="nsew")
        self.event_list = tk.Listbox(log_box, activestyle="none")
        self.event_list.pack(fill=tk.BOTH, expand=True)

        control_box = ttk.LabelFrame(right, text="Control", padding=GROUP_PADDING)
        control_box.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        control_box.columnconfigure(0, weight=1)

        telemetry_row = ttk.Frame(control_box)
        telemetry_row.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        telemetry_row.columnconfigure(0, weight=1)
        ttk.Label(telemetry_row, text="Telemetry panel").grid(row=0, column=0, sticky="w")
        self.telemetry_toggle_button = ttk.Button(
            telemetry_row,
            text="Hide telemetry",
            command=self._toggle_telemetry_panel,
        )
        self.telemetry_toggle_button.grid(row=0, column=1, sticky="e")

        stack_row = ttk.LabelFrame(control_box, text="Simulation Stack", padding=INNER_PADDING)
        stack_row.grid(row=1, column=0, sticky="ew", pady=(0, 4))
        stack_row.columnconfigure(0, weight=1)
        stack_buttons = ttk.Frame(stack_row)
        stack_buttons.grid(row=0, column=0, sticky="ew")
        ttk.Button(stack_buttons, text="Start SITL/MuJoCo", command=self._start_sim_stack).pack(side=tk.LEFT)
        ttk.Button(stack_buttons, text="Stop/Reset", command=self._stop_sim_stack).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(
            stack_buttons,
            text="Ping360 panel",
            style="Info.TButton",
            command=self._toggle_ping360_window,
        ).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Label(stack_row, textvariable=self.sim_stack_status_var, anchor="w").grid(
            row=1, column=0, sticky="ew", pady=(2, 0)
        )

        arm_row = ttk.Frame(control_box)
        arm_row.grid(row=2, column=0, sticky="ew", pady=(0, 4))
        ttk.Button(arm_row, text="Arm", command=lambda: self.node.arm(True)).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(arm_row, text="Disarm", command=lambda: self.node.arm(False)).pack(side=tk.LEFT)

        mode_row = ttk.LabelFrame(control_box, text="Modes", padding=INNER_PADDING)
        mode_row.grid(row=3, column=0, sticky="ew", pady=(0, 4))
        for idx, mode in enumerate(self.MODE_BUTTONS):
            ttk.Button(mode_row, text=mode, command=lambda m=mode: self.node.set_mode(m)).grid(
                row=idx // 3, column=idx % 3, sticky="ew", padx=3, pady=3
            )
        for col in range(3):
            mode_row.columnconfigure(col, weight=1)

        replay_box = ttk.LabelFrame(control_box, text="RC Override Replay", padding=INNER_PADDING)
        replay_box.grid(row=4, column=0, sticky="ew", pady=(0, 4))
        replay_box.columnconfigure(0, weight=1)

        replay_path_row = ttk.Frame(replay_box)
        replay_path_row.grid(row=0, column=0, sticky="ew")
        replay_path_row.columnconfigure(0, weight=1)
        ttk.Entry(replay_path_row, textvariable=self.rc_replay_path_var).grid(
            row=0, column=0, sticky="ew", padx=(0, 4)
        )
        ttk.Button(replay_path_row, text="Browse", command=self._browse_rc_replay_bag).grid(row=0, column=1)

        timeline_row = ttk.Frame(replay_box)
        timeline_row.grid(row=1, column=0, sticky="ew", pady=(4, 0))
        timeline_row.columnconfigure(0, weight=1)
        self.rc_replay_slider = ttk.Scale(
            timeline_row,
            from_=0.0,
            to=1.0,
            orient=tk.HORIZONTAL,
            variable=self.rc_replay_position_var,
            command=self._on_rc_replay_slider_changed,
        )
        self.rc_replay_slider.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.rc_replay_slider.state(["disabled"])
        self.rc_replay_slider.bind("<Button-1>", self._on_rc_replay_slider_press)
        self.rc_replay_slider.bind("<B1-Motion>", self._on_rc_replay_slider_motion)
        self.rc_replay_slider.bind("<ButtonRelease-1>", self._on_rc_replay_slider_release)
        ttk.Label(timeline_row, textvariable=self.rc_replay_time_var, width=15, anchor="e").grid(
            row=0, column=1, sticky="e"
        )

        replay_controls = ttk.Frame(replay_box)
        replay_controls.grid(row=2, column=0, sticky="ew", pady=(3, 0))
        ttk.Button(replay_controls, text="Load", command=self._load_rc_replay).pack(side=tk.LEFT)
        self.rc_replay_play_button = ttk.Button(replay_controls, text="Play", command=self._start_rc_replay)
        self.rc_replay_play_button.pack(side=tk.LEFT, padx=(4, 0))
        self.rc_replay_pause_button = ttk.Button(
            replay_controls,
            text="Pause",
            command=self._toggle_rc_replay_pause,
        )
        self.rc_replay_pause_button.pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(replay_controls, text="Stop", command=self._stop_rc_replay).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Label(replay_controls, text="rate").pack(side=tk.LEFT, padx=(10, 2))
        ttk.Entry(replay_controls, width=4, textvariable=self.rc_replay_rate_var).pack(side=tk.LEFT)

        ttk.Label(replay_box, textvariable=self.rc_replay_status_var, anchor="w").grid(
            row=3, column=0, sticky="ew", pady=(2, 0)
        )

        physics_row = ttk.LabelFrame(control_box, text="Sim Param Tuning", padding=INNER_PADDING)
        physics_row.grid(row=5, column=0, sticky="ew", pady=(0, 4))
        physics_row.columnconfigure(0, weight=1)
        ttk.Label(
            physics_row,
            textvariable=self.physics_status_var,
            anchor="w",
        ).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.physics_toggle_button = ttk.Button(
            physics_row,
            text="Open physics params",
            style="Accent.TButton",
            command=self._show_physics_window,
        )
        self.physics_toggle_button.grid(row=0, column=1, sticky="e")
        self._load_physics_params_into_fields(silent=True)

        rc_box = ttk.LabelFrame(control_box, text="RC Override Joysticks", padding=INNER_PADDING)
        rc_box.grid(row=6, column=0, sticky="ew", pady=(0, 4))
        rc_box.columnconfigure(0, weight=1)

        rc_header = ttk.Frame(rc_box)
        rc_header.grid(row=0, column=0, sticky="ew", pady=(0, 3))
        ttk.Checkbutton(
            rc_header,
            text="Enable RC override",
            variable=self.rc_override_enabled,
            command=self._on_rc_override_toggle,
        ).pack(side=tk.LEFT)
        ttk.Button(rc_header, text="Center", command=self._center_rc_sticks).pack(side=tk.RIGHT)
        ttk.Button(rc_header, text="Release RC", command=self._release_rc_override).pack(side=tk.RIGHT, padx=(0, 6))

        stick_row = ttk.Frame(rc_box)
        stick_row.grid(row=1, column=0, sticky="ew")
        stick_row.columnconfigure(0, weight=1)
        stick_row.columnconfigure(1, weight=1)

        self.left_stick = VirtualJoystick(
            stick_row,
            title="Left Stick",
            x_var=self.rc_yaw_var,
            y_var=self.rc_heave_var,
            x_label="yaw",
            y_label="heave",
        )
        self.left_stick.grid(row=0, column=0, sticky="nsew", padx=(0, 3))

        self.right_stick = VirtualJoystick(
            stick_row,
            title="Right Stick",
            x_var=self.rc_lateral_var,
            y_var=self.rc_forward_var,
            x_label="lateral",
            y_label="forward",
        )
        self.right_stick.grid(row=0, column=1, sticky="nsew", padx=(3, 0))

        details_row = ttk.Frame(control_box)
        details_row.grid(row=8, column=0, sticky="ew", pady=(4, 0))
        self.control_details_button = ttk.Button(
            details_row,
            text="Show control details",
            command=self._toggle_control_details,
        )
        self.control_details_button.pack(side=tk.RIGHT)

        self.control_details_frame = ttk.LabelFrame(control_box, text="Control Details", padding=INNER_PADDING)
        self.control_details_frame.grid(row=9, column=0, sticky="ew", pady=(3, 0))
        self.control_details_frame.columnconfigure(0, weight=1)
        ttk.Label(self.control_details_frame, textvariable=self.control_var, anchor="w").grid(
            row=0, column=0, sticky="ew", pady=1
        )
        ttk.Label(self.control_details_frame, textvariable=self.rc_override_var, anchor="w").grid(
            row=1, column=0, sticky="ew", pady=1
        )
        self.control_details_frame.grid_remove()

    def _set_rc_replay_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.rc_replay_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.rc_replay_status_var.set(text))
        except Exception:
            pass

    def _set_rc_replay_pause_button(self, text: str) -> None:
        try:
            self.root.after(0, lambda: self.rc_replay_pause_button.config(text=text))
        except Exception:
            pass

    def _rc_replay_running(self) -> bool:
        return self._rc_replay_thread is not None and self._rc_replay_thread.is_alive()

    def _rc_replay_sample_index_for_time(self, time_s: float) -> int:
        if not self._rc_replay_samples:
            return 0
        target = clamp(float(time_s), 0.0, self._rc_replay_duration_s)
        lo = 0
        hi = len(self._rc_replay_samples)
        while lo < hi:
            mid = (lo + hi) // 2
            if self._rc_replay_samples[mid].time_s < target:
                lo = mid + 1
            else:
                hi = mid
        return min(lo, len(self._rc_replay_samples) - 1)

    def _set_rc_replay_position(self, time_s: float, *, force: bool = False) -> None:
        time_s = clamp(float(time_s), 0.0, max(self._rc_replay_duration_s, 0.0))

        def apply() -> None:
            if self._rc_replay_slider_dragging and not force:
                return
            self.rc_replay_position_var.set(time_s)
            self._update_rc_replay_time_label(time_s)

        if threading.current_thread() is threading.main_thread():
            apply()
        else:
            try:
                self.root.after(0, apply)
            except Exception:
                pass

    def _update_rc_replay_time_label(self, time_s: float | None = None) -> None:
        if time_s is None:
            try:
                time_s = float(self.rc_replay_position_var.get())
            except Exception:
                time_s = 0.0
        self.rc_replay_time_var.set(
            f"{format_replay_time(time_s)} / {format_replay_time(self._rc_replay_duration_s)}"
        )

    def _event_to_rc_replay_time(self, event) -> float:
        width = max(int(self.rc_replay_slider.winfo_width()), 1)
        ratio = clamp(float(event.x) / float(width), 0.0, 1.0)
        return ratio * max(self._rc_replay_duration_s, 0.0)

    def _set_replay_slider_from_event(self, event) -> float:
        time_s = self._event_to_rc_replay_time(event)
        self.rc_replay_position_var.set(time_s)
        self._update_rc_replay_time_label(time_s)
        return time_s

    def _on_rc_replay_slider_changed(self, value: str) -> None:
        if self._rc_replay_slider_dragging:
            return
        try:
            time_s = float(value)
        except (TypeError, ValueError):
            return
        self._update_rc_replay_time_label(time_s)

    def _on_rc_replay_slider_press(self, event):
        if not self._rc_replay_samples:
            return "break"
        self._rc_replay_slider_dragging = True
        self._set_replay_slider_from_event(event)
        return "break"

    def _on_rc_replay_slider_motion(self, event):
        if not self._rc_replay_samples:
            return "break"
        self._set_replay_slider_from_event(event)
        return "break"

    def _on_rc_replay_slider_release(self, event):
        if not self._rc_replay_samples:
            self._rc_replay_slider_dragging = False
            return "break"
        time_s = self._set_replay_slider_from_event(event)
        self._rc_replay_slider_dragging = False
        self._request_rc_replay_seek(time_s)
        return "break"

    def _request_rc_replay_seek(self, time_s: float) -> None:
        time_s = clamp(float(time_s), 0.0, max(self._rc_replay_duration_s, 0.0))
        with self._rc_replay_seek_lock:
            self._rc_replay_seek_time_s = time_s
        if not self._rc_replay_running():
            self._set_rc_replay_position(time_s, force=True)
            self._set_rc_replay_status(f"replay: seek {format_replay_time(time_s)}")

    def _consume_rc_replay_seek(self) -> float | None:
        with self._rc_replay_seek_lock:
            time_s = self._rc_replay_seek_time_s
            self._rc_replay_seek_time_s = None
        return time_s

    def _browse_rc_replay_bag(self) -> None:
        initial_dir = str(DEFAULT_RC_REPLAY_BAG.parent if DEFAULT_RC_REPLAY_BAG.parent.exists() else APP_ROOT)
        selected = filedialog.askdirectory(
            parent=self.root,
            title="Select ROS2 bag directory containing /mavros/rc/override",
            initialdir=initial_dir,
        )
        if selected:
            self.rc_replay_path_var.set(selected)
            self._set_rc_replay_status("replay: selected, not loaded")

    def _load_rc_replay(self) -> bool:
        if self._rc_replay_running():
            self._set_rc_replay_status("replay: stop current playback before loading")
            return False
        try:
            samples = load_rc_override_replay(self.rc_replay_path_var.get())
        except Exception as exc:
            self._rc_replay_samples = []
            self._rc_replay_duration_s = 0.0
            self.rc_replay_slider.configure(to=1.0)
            self.rc_replay_slider.state(["disabled"])
            self._set_rc_replay_position(0.0, force=True)
            self._set_rc_replay_status(f"replay load failed: {exc}")
            return False

        self._rc_replay_samples = samples
        duration = samples[-1].time_s if samples else 0.0
        self._rc_replay_duration_s = duration
        self.rc_replay_slider.configure(to=max(duration, 1e-6))
        self.rc_replay_slider.state(["!disabled"])
        self._set_rc_replay_position(0.0, force=True)
        with self._rc_replay_seek_lock:
            self._rc_replay_seek_time_s = None
        self._set_rc_replay_status(
            f"replay loaded: {len(samples)} msgs, {format_replay_time(duration)}"
        )
        return True

    def _rc_replay_rate(self) -> float:
        try:
            rate = float(self.rc_replay_rate_var.get())
        except ValueError:
            rate = 1.0
        rate = clamp(rate, 0.1, 5.0)
        self.rc_replay_rate_var.set(f"{rate:g}")
        return rate

    def _start_rc_replay(self) -> None:
        if self._rc_replay_running():
            return
        if not self._rc_replay_samples and not self._load_rc_replay():
            return

        self.rc_override_enabled.set(False)
        self._center_rc_sticks()
        self._rc_override_prev = False

        self._rc_replay_stop_event.clear()
        self._rc_replay_pause_event.clear()
        self.rc_replay_pause_button.config(text="Pause")
        rate = self._rc_replay_rate()
        start_time_s = clamp(
            float(self.rc_replay_position_var.get()),
            0.0,
            max(self._rc_replay_duration_s, 0.0),
        )
        if start_time_s >= max(self._rc_replay_duration_s - 0.01, 0.0):
            start_time_s = 0.0
            self._set_rc_replay_position(0.0, force=True)
        samples = list(self._rc_replay_samples)
        self._rc_replay_thread = threading.Thread(
            target=self._run_rc_replay,
            args=(samples, rate, start_time_s),
            daemon=True,
        )
        self._rc_replay_thread.start()

    def _toggle_rc_replay_pause(self) -> None:
        if not self._rc_replay_running():
            return
        if self._rc_replay_pause_event.is_set():
            self._rc_replay_pause_event.clear()
            self.rc_replay_pause_button.config(text="Pause")
            self._set_rc_replay_status("replay: running")
        else:
            self._rc_replay_pause_event.set()
            self.rc_replay_pause_button.config(text="Resume")
            self._set_rc_replay_status("replay: paused")

    def _stop_rc_replay(self) -> None:
        running = self._rc_replay_running()
        self._rc_replay_stop_event.set()
        self._rc_replay_pause_event.clear()
        self.rc_replay_pause_button.config(text="Pause")
        self.node.publish_rc_release()
        self._guided_control_prev = False
        self._rc_override_prev = False
        self._set_rc_replay_status("replay: stopping" if running else "replay: stopped")

    def _run_rc_replay(self, samples: list[RcReplaySample], rate: float, start_time_s: float) -> None:
        local_duration_s = samples[-1].time_s if samples else 0.0
        idx = self._rc_replay_sample_index_for_time(start_time_s)
        start_wall = time.monotonic() - start_time_s / rate
        paused_since: float | None = None
        last_status_wall = 0.0
        last_position_wall = 0.0
        stopped = False

        while idx < len(samples):
            if self._rc_replay_stop_event.is_set():
                stopped = True
                break

            seek_time_s = self._consume_rc_replay_seek()
            if seek_time_s is not None:
                idx = self._rc_replay_sample_index_for_time(seek_time_s)
                start_wall = time.monotonic() - seek_time_s / rate
                self._set_rc_replay_status(f"replay: seek {format_replay_time(seek_time_s)}")
                continue

            while self._rc_replay_pause_event.is_set() and not self._rc_replay_stop_event.is_set():
                if paused_since is None:
                    paused_since = time.monotonic()
                seek_time_s = self._consume_rc_replay_seek()
                if seek_time_s is not None:
                    idx = self._rc_replay_sample_index_for_time(seek_time_s)
                    start_wall = time.monotonic() - seek_time_s / rate
                    paused_since = time.monotonic()
                    self._set_rc_replay_position(seek_time_s, force=False)
                    self._set_rc_replay_status(f"replay: paused at {format_replay_time(seek_time_s)}")
                time.sleep(0.03)
            if paused_since is not None:
                start_wall += time.monotonic() - paused_since
                paused_since = None
            if self._rc_replay_stop_event.is_set():
                stopped = True
                break

            sample = samples[idx]
            target_wall = start_wall + sample.time_s / rate
            seek_applied = False
            while not self._rc_replay_stop_event.is_set():
                seek_time_s = self._consume_rc_replay_seek()
                if seek_time_s is not None:
                    idx = self._rc_replay_sample_index_for_time(seek_time_s)
                    start_wall = time.monotonic() - seek_time_s / rate
                    self._set_rc_replay_status(f"replay: seek {format_replay_time(seek_time_s)}")
                    seek_applied = True
                    break
                remaining = target_wall - time.monotonic()
                if remaining <= 0.0:
                    break
                now = time.monotonic()
                if now - last_position_wall > 0.15:
                    elapsed_s = clamp((now - start_wall) * rate, 0.0, local_duration_s)
                    self._set_rc_replay_position(elapsed_s)
                    last_position_wall = now
                time.sleep(min(remaining, 0.02))
            if idx >= len(samples):
                break
            if seek_applied:
                continue
            if self._rc_replay_stop_event.is_set():
                stopped = True
                break

            if not self.node.publish_rc_channels(sample.channels):
                stopped = True
                self._set_rc_replay_status("replay failed: rc override publisher unavailable")
                break

            now = time.monotonic()
            if now - last_position_wall > 0.15:
                self._set_rc_replay_position(sample.time_s)
                last_position_wall = now
            if now - last_status_wall > 0.5:
                last_status_wall = now
                self._set_rc_replay_status(
                    f"replay: {idx + 1}/{len(samples)}  {format_replay_time(sample.time_s)}  rate={rate:g}x"
                )
            idx += 1

        self.node.publish_rc_release()
        self._rc_replay_stop_event.clear()
        self._rc_replay_pause_event.clear()
        self._set_rc_replay_pause_button("Pause")
        if stopped:
            self._set_rc_replay_status("replay: stopped")
        else:
            self._set_rc_replay_position(local_duration_s, force=True)
            self._set_rc_replay_status("replay: finished")

    def _zero_controls(self) -> None:
        self._center_rc_sticks()

    def _center_rc_sticks(self) -> None:
        self.rc_forward_var.set(0.0)
        self.rc_lateral_var.set(0.0)
        self.rc_heave_var.set(0.0)
        self.rc_yaw_var.set(0.0)

    def _release_rc_override(self) -> None:
        self.rc_override_enabled.set(False)
        self._center_rc_sticks()
        self.node.publish_rc_release()

    def _toggle_control_details(self) -> None:
        show = not self.control_details_visible.get()
        self.control_details_visible.set(show)
        if show:
            self.control_details_frame.grid()
            self.control_details_button.config(text="Hide control details")
        else:
            self.control_details_frame.grid_remove()
            self.control_details_button.config(text="Show control details")

    def _toggle_vehicle_details(self) -> None:
        show = not self.vehicle_details_visible.get()
        self.vehicle_details_visible.set(show)
        if show:
            self.vehicle_details_frame.grid()
            self.vehicle_details_button.config(text="Details v")
        else:
            self.vehicle_details_frame.grid_remove()
            self.vehicle_details_button.config(text="Details >")

    def _toggle_telemetry_panel(self) -> None:
        show = not self.telemetry_visible.get()
        self.telemetry_visible.set(show)
        if self.main_container is None or self.telemetry_panel is None or self.control_panel is None:
            return

        if show:
            self.root.minsize(*WINDOW_MINSIZE)
            self.telemetry_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
            self.control_panel.grid_configure(row=0, column=1, columnspan=1, sticky="nsew")
            self.main_container.columnconfigure(0, weight=3)
            self.main_container.columnconfigure(1, weight=2)
            if self.root.winfo_width() < WINDOW_MINSIZE[0]:
                self.root.geometry(f"{WINDOW_MINSIZE[0]}x{max(self.root.winfo_height(), WINDOW_MINSIZE[1])}")
            self.telemetry_toggle_button.config(text="Hide telemetry")
        else:
            self.telemetry_panel.grid_remove()
            self.control_panel.grid_configure(row=0, column=0, columnspan=2, sticky="nsew")
            self.main_container.columnconfigure(0, weight=1)
            self.main_container.columnconfigure(1, weight=0)
            self.root.minsize(*TELEMETRY_HIDDEN_MINSIZE)
            self.root.geometry(
                f"{TELEMETRY_HIDDEN_WIDTH}x{max(self.root.winfo_height(), TELEMETRY_HIDDEN_MINSIZE[1])}"
            )
            self.telemetry_toggle_button.config(text="Show telemetry")

    def _sim_stack_running(self) -> bool:
        return self._sim_stack_process is not None and self._sim_stack_process.poll() is None

    def _set_sim_stack_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.sim_stack_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.sim_stack_status_var.set(text))
        except Exception:
            pass

    def _toggle_ping360_window(self) -> None:
        if self.ping360_window is not None and self.ping360_window.winfo_exists():
            self._close_ping360_window()
            return
        self._show_ping360_window()

    def _show_ping360_window(self) -> None:
        if self.ping360_window is not None and self.ping360_window.winfo_exists():
            self.ping360_window.deiconify()
            self.ping360_window.lift()
            return

        win = tk.Toplevel(self.root)
        win.title("Ping360 Control")
        win.geometry("520x300")
        win.minsize(460, 260)
        win.protocol("WM_DELETE_WINDOW", self._close_ping360_window)
        self.ping360_window = win

        outer = ttk.Frame(win, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)

        status_box = ttk.LabelFrame(outer, text="Status", padding=INNER_PADDING)
        status_box.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        status_box.columnconfigure(0, weight=1)
        ttk.Label(status_box, textvariable=self.ping360_view_status_var, anchor="w").grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Label(status_box, textvariable=self.ping360_summary_var, anchor="w").grid(
            row=1, column=0, sticky="ew", pady=(2, 0)
        )

        view_box = ttk.LabelFrame(outer, text="Viewer", padding=INNER_PADDING)
        view_box.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(view_box, text="Open RViz/rqt", style="Info.TButton", command=self._start_ping360_view).pack(
            side=tk.LEFT
        )
        ttk.Button(view_box, text="Close Viewer", command=self._stop_ping360_view).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        params = ttk.LabelFrame(outer, text="Ping360 Params", padding=INNER_PADDING)
        params.grid(row=2, column=0, sticky="ew", pady=(0, 6))
        for col in range(8):
            params.columnconfigure(col, weight=1 if col in (1, 3, 5, 7) else 0)

        ttk.Label(params, text="range").grid(row=0, column=0, sticky="w")
        ttk.Entry(params, width=6, textvariable=self.ping360_range_var).grid(
            row=0, column=1, sticky="ew", padx=(2, 6)
        )
        ttk.Label(params, text="step").grid(row=0, column=2, sticky="w")
        ttk.Entry(params, width=4, textvariable=self.ping360_num_steps_var).grid(
            row=0, column=3, sticky="ew", padx=(2, 6)
        )
        ttk.Label(params, text="gain").grid(row=0, column=4, sticky="w")
        ttk.Entry(params, width=4, textvariable=self.ping360_gain_var).grid(
            row=0, column=5, sticky="ew", padx=(2, 6)
        )
        ttk.Label(params, text="link").grid(row=0, column=6, sticky="w")
        ttk.Combobox(
            params,
            textvariable=self.ping360_interface_var,
            values=("ethernet", "usb", "rs485"),
            state="readonly",
            width=8,
        ).grid(row=0, column=7, sticky="ew", padx=(2, 0))

        ttk.Label(params, text="kHz").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(params, width=6, textvariable=self.ping360_frequency_var).grid(
            row=1, column=1, sticky="ew", padx=(2, 6), pady=(4, 0)
        )
        ttk.Label(params, text="start").grid(row=1, column=2, sticky="w", pady=(4, 0))
        ttk.Entry(params, width=5, textvariable=self.ping360_start_angle_var).grid(
            row=1, column=3, sticky="ew", padx=(2, 6), pady=(4, 0)
        )
        ttk.Label(params, text="stop").grid(row=1, column=4, sticky="w", pady=(4, 0))
        ttk.Entry(params, width=5, textvariable=self.ping360_stop_angle_var).grid(
            row=1, column=5, sticky="ew", padx=(2, 6), pady=(4, 0)
        )
        ttk.Label(params, text="grad").grid(row=1, column=6, sticky="w", pady=(4, 0))
        ttk.Button(params, text="Apply", style="Info.TButton", command=self._apply_ping360_params).grid(
            row=1, column=7, sticky="ew", padx=(2, 0), pady=(4, 0)
        )

        footer = ttk.Frame(outer)
        footer.grid(row=3, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)
        ttk.Label(
            footer,
            text="Publishes /ping360/config and opens the configured Ping360 view.",
            anchor="w",
            foreground="#64748b",
        ).grid(row=0, column=0, sticky="ew")
        ttk.Button(footer, text="Close", command=self._close_ping360_window).grid(row=0, column=1, padx=(8, 0))

    def _close_ping360_window(self) -> None:
        if self.ping360_window is not None and self.ping360_window.winfo_exists():
            self.ping360_window.destroy()
        self.ping360_window = None

    def _ping360_view_running(self) -> bool:
        return self._ping360_view_process is not None and self._ping360_view_process.poll() is None

    def _set_ping360_view_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.ping360_view_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.ping360_view_status_var.set(text))
        except Exception:
            pass

    def _start_ping360_view(self) -> None:
        if self._ping360_view_running():
            self._set_ping360_view_status("ping360 view: already open")
            return

        rviz2 = shutil.which("rviz2")
        ros2 = shutil.which("ros2")
        if rviz2 and PING360_RVIZ_CONFIG.exists():
            cmd = [rviz2, "-d", str(PING360_RVIZ_CONFIG)]
            label = "rviz2"
        elif ros2:
            cmd = [ros2, "run", "rqt_image_view", "rqt_image_view", "/ping360/image"]
            label = "rqt_image_view"
        else:
            self._set_ping360_view_status("ping360 view failed: rviz2/ros2 not found")
            return

        log_dir = SIM_STACK_DIR / "logs"
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"ping360_view_{_dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            log_file = log_path.open("w", encoding="utf-8")
        except Exception as exc:
            self._set_ping360_view_status(f"ping360 log open failed: {exc}")
            return

        try:
            proc = subprocess.Popen(
                cmd,
                cwd=str(APP_ROOT),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except Exception as exc:
            log_file.close()
            self._set_ping360_view_status(f"ping360 view start failed: {exc}")
            return
        finally:
            try:
                log_file.close()
            except Exception:
                pass

        self._ping360_view_process = proc
        self._ping360_view_log_path = log_path
        self._set_ping360_view_status(f"ping360 view: {label} open")
        self.node.push_event(f"ping360 view opened: {label}, log={log_path.name}")

    def _stop_ping360_view(self) -> None:
        proc = self._ping360_view_process
        if proc is None or proc.poll() is not None:
            self._set_ping360_view_status("ping360 view: closed")
            self._ping360_view_process = None
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
        self._set_ping360_view_status("ping360 view: closing")
        self.node.push_event("ping360 view close requested")

    def _apply_ping360_params(self) -> None:
        range_m = self._read_float_var(self.ping360_range_var, 2.0, 0.75, 50.0)
        num_steps = self._read_int_var(self.ping360_num_steps_var, 1, 1, 10)
        gain = self._read_int_var(self.ping360_gain_var, 0, 0, 2)
        frequency_khz = self._read_int_var(self.ping360_frequency_var, 750, 500, 1000)
        start_grad = self._read_int_var(self.ping360_start_angle_var, 0, 0, 399)
        stop_grad = self._read_int_var(self.ping360_stop_angle_var, 399, 0, 399)
        interface_mode = self.ping360_interface_var.get().strip().lower()
        if interface_mode not in {"ethernet", "usb", "rs485"}:
            interface_mode = "ethernet"
            self.ping360_interface_var.set(interface_mode)

        self.node.publish_ping360_config(
            range_m=range_m,
            num_steps=num_steps,
            gain=gain,
            interface_mode=interface_mode,
            frequency_khz=frequency_khz,
            start_angle_grad=start_grad,
            stop_angle_grad=stop_grad,
        )
        self._set_ping360_view_status("ping360 config: published")

    def _start_sim_stack(self) -> None:
        if self._sim_stack_running():
            self._set_sim_stack_status("sim: already running")
            return
        if not START_SIM_STACK_SCRIPT.exists():
            self._set_sim_stack_status(f"sim script missing: {START_SIM_STACK_SCRIPT}")
            return
        if not os.access(START_SIM_STACK_SCRIPT, os.X_OK):
            self._set_sim_stack_status(f"sim script is not executable: {START_SIM_STACK_SCRIPT}")
            return

        if self._rc_replay_running():
            self._stop_rc_replay()
        self.rc_override_enabled.set(False)
        self.node.publish_rc_release()

        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        # GUI-started SITL runs should go through ArduSub, not also inject the
        # same RC override directly into MuJoCo.
        env["ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK"] = "0"
        log_dir = SIM_STACK_DIR / "logs"
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"gui_start_stack_{_dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            log_file = log_path.open("w", encoding="utf-8")
        except Exception as exc:
            self._set_sim_stack_status(f"sim log open failed: {exc}")
            return

        try:
            proc = subprocess.Popen(
                [str(START_SIM_STACK_SCRIPT)],
                cwd=str(SIM_STACK_DIR),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
                start_new_session=True,
            )
        except Exception as exc:
            log_file.close()
            self._set_sim_stack_status(f"sim start failed: {exc}")
            return
        finally:
            try:
                log_file.close()
            except Exception:
                pass

        self._sim_stack_process = proc
        self._sim_stack_log_path = log_path
        self._set_sim_stack_status("sim: starting SITL/MuJoCo")
        self.node.push_event(f"sim stack start requested: {log_path.name}")
        self._sim_stack_thread = threading.Thread(
            target=self._watch_sim_stack_output,
            args=(proc, log_path),
            daemon=True,
        )
        self._sim_stack_thread.start()

    def _watch_sim_stack_output(self, proc: subprocess.Popen[str], log_path: Path) -> None:
        last_line = ""
        try:
            with log_path.open("r", encoding="utf-8", errors="replace") as log_stream:
                while True:
                    raw_line = log_stream.readline()
                    if raw_line:
                        line = raw_line.strip()
                        if not line:
                            continue
                        last_line = line
                        if line.startswith(
                            ("[start]", "[reset]", "[launch]", "[runtime]", "[physics]", "[sitl]", "[model]")
                        ):
                            short_line = line if len(line) <= 150 else f"{line[:147]}..."
                            self.node.push_event(short_line)
                            if line.startswith(("[start]", "[reset]")):
                                self._set_sim_stack_status(f"sim: {short_line}")
                        continue
                    if proc.poll() is not None:
                        break
                    time.sleep(0.1)
                for raw_line in log_stream:
                    line = raw_line.strip()
                    if line:
                        last_line = line
            rc = proc.returncode if proc.returncode is not None else proc.wait()
        except Exception as exc:
            rc = -1
            last_line = f"reader failed: {exc}"

        def finish() -> None:
            if self._sim_stack_process is proc:
                self._sim_stack_process = None
            if rc == 0:
                self.sim_stack_status_var.set("sim: exited")
                self.node.push_event("sim stack exited")
            elif rc < 0:
                self.sim_stack_status_var.set("sim: stopped")
                self.node.push_event("sim stack stopped")
            else:
                text = last_line if last_line else f"rc={rc}"
                self.sim_stack_status_var.set(f"sim failed: {text}")
                self.node.push_event(f"sim stack failed: {text}")

        try:
            self.root.after(0, finish)
        except Exception:
            pass

    def _terminate_sim_stack_process(self) -> None:
        proc = self._sim_stack_process
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass

    def _stop_sim_stack(self) -> None:
        self._terminate_sim_stack_process()
        self._set_sim_stack_status("sim: resetting stack")
        self.node.push_event("sim stack reset requested")
        if self._sim_stack_reset_thread is not None and self._sim_stack_reset_thread.is_alive():
            return
        self._sim_stack_reset_thread = threading.Thread(target=self._run_sim_stack_reset, daemon=True)
        self._sim_stack_reset_thread.start()

    def _run_sim_stack_reset(self) -> None:
        if not RESET_SIM_STACK_SCRIPT.exists():
            self._set_sim_stack_status(f"reset script missing: {RESET_SIM_STACK_SCRIPT}")
            return
        cmd = [str(RESET_SIM_STACK_SCRIPT), "--wipe-eeprom"]
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=str(SIM_STACK_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            assert proc.stdout is not None
            for raw_line in proc.stdout:
                line = raw_line.strip()
                if not line:
                    continue
                if line.startswith("[reset]"):
                    short_line = line if len(line) <= 150 else f"{line[:147]}..."
                    self.node.push_event(short_line)
                    self._set_sim_stack_status(f"sim: {short_line}")
            rc = proc.wait()
        except Exception as exc:
            self._set_sim_stack_status(f"sim reset failed: {exc}")
            self.node.push_event(f"sim reset failed: {exc}")
            return
        if rc == 0:
            self._set_sim_stack_status("sim: stopped/reset")
            self.node.push_event("sim stack stopped/reset")
        else:
            self._set_sim_stack_status(f"sim reset failed: rc={rc}")
            self.node.push_event(f"sim reset failed: rc={rc}")

    def _show_physics_window(self) -> None:
        if self.physics_window is not None and self.physics_window.winfo_exists():
            self._load_physics_params_into_fields(silent=True)
            self.physics_window.deiconify()
            self.physics_window.lift()
            return

        self._load_physics_params_into_fields(silent=True)
        win = tk.Toplevel(self.root)
        win.title("UUV Sim Param Tuning")
        win.geometry("820x640")
        win.minsize(720, 500)
        win.protocol("WM_DELETE_WINDOW", self._close_physics_window)
        self.physics_window = win

        outer = ttk.Frame(win, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(
            header,
            text=f"profile: {PHYSICS_PROFILE_NAME}   file: {PHYSICS_PROFILE_PATH}",
            anchor="w",
        ).grid(row=0, column=0, sticky="ew")
        ttk.Button(header, text="Reload", command=self._load_physics_params_into_fields).grid(
            row=0, column=1, padx=(8, 0)
        )
        ttk.Button(header, text="Apply", command=self._apply_physics_params).grid(
            row=0, column=2, padx=(4, 0)
        )
        ttk.Button(
            header,
            text="Apply + Restart",
            command=lambda: self._apply_physics_params(restart=True),
        ).grid(row=0, column=3, padx=(4, 0))

        ttk.Label(
            outer,
            textvariable=self.physics_status_var,
            anchor="w",
        ).grid(row=1, column=0, sticky="ew", pady=(6, 6))

        body = ttk.Frame(outer)
        body.grid(row=2, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)

        canvas = tk.Canvas(body, highlightthickness=0)
        scroll_y = ttk.Scrollbar(body, orient=tk.VERTICAL, command=canvas.yview)
        canvas.configure(yscrollcommand=scroll_y.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        self.physics_canvas = canvas

        grid = ttk.Frame(canvas, padding=(0, 0, 6, 0))
        self.physics_scroll_frame = grid
        window_id = canvas.create_window((0, 0), window=grid, anchor="nw")
        grid.columnconfigure(1, weight=1)
        grid.columnconfigure(3, weight=2)

        def update_scroll_region(_event: tk.Event | None = None) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))

        def update_inner_width(event: tk.Event) -> None:
            canvas.itemconfigure(window_id, width=event.width)

        grid.bind("<Configure>", update_scroll_region)
        canvas.bind("<Configure>", update_inner_width)

        ttk.Label(grid, text="parameter", anchor="w").grid(row=0, column=0, sticky="ew", pady=(0, 4))
        ttk.Label(grid, text="value", anchor="w").grid(row=0, column=1, sticky="ew", pady=(0, 4))
        ttk.Label(grid, text="key", anchor="w").grid(row=0, column=2, sticky="ew", padx=(8, 8), pady=(0, 4))
        ttk.Label(grid, text="what it changes", anchor="w").grid(row=0, column=3, sticky="ew", pady=(0, 4))

        for row_idx, spec in enumerate(PHYSICS_PARAM_SPECS, start=1):
            key = str(spec["key"])
            ttk.Label(grid, text=str(spec["label"]), width=25, anchor="w").grid(
                row=row_idx, column=0, sticky="w", pady=2
            )
            ttk.Entry(grid, textvariable=self.physics_param_vars[key], width=24).grid(
                row=row_idx, column=1, sticky="ew", padx=(4, 8), pady=2
            )
            ttk.Label(grid, text=key, anchor="w", foreground="#64748b").grid(
                row=row_idx, column=2, sticky="w", padx=(0, 8), pady=2
            )
            ttk.Label(
                grid,
                text=str(spec["description"]),
                anchor="w",
                foreground="#475569",
                wraplength=300,
            ).grid(row=row_idx, column=3, sticky="ew", pady=2)

        footer = ttk.Frame(outer)
        footer.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        footer.columnconfigure(0, weight=1)
        ttk.Label(
            footer,
            text="Changes are written to sim_profiles.json. Running MuJoCo must be restarted to reload physics parameters.",
            anchor="w",
            foreground="#64748b",
        ).grid(row=0, column=0, sticky="ew")
        ttk.Button(footer, text="Close", command=self._close_physics_window).grid(row=0, column=1, padx=(8, 0))

    def _close_physics_window(self) -> None:
        if self.physics_window is not None and self.physics_window.winfo_exists():
            self.physics_window.destroy()
        self.physics_window = None
        self.physics_canvas = None
        self.physics_scroll_frame = None

    def _set_physics_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.physics_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.physics_status_var.set(text))
        except Exception:
            pass

    @staticmethod
    def _format_physics_value(value: Any) -> str:
        if isinstance(value, (list, tuple)):
            return " ".join(f"{float(item):g}" for item in value)
        return f"{float(value):g}"

    @staticmethod
    def _get_nested_value(mapping: dict[str, Any], dotted_key: str) -> Any:
        current: Any = mapping
        for part in dotted_key.split("."):
            if not isinstance(current, dict) or part not in current:
                return None
            current = current[part]
        return current

    @staticmethod
    def _set_nested_value(mapping: dict[str, Any], dotted_key: str, value: Any) -> None:
        current = mapping
        parts = dotted_key.split(".")
        for part in parts[:-1]:
            child = current.get(part)
            if not isinstance(child, dict):
                child = {}
                current[part] = child
            current = child
        current[parts[-1]] = value

    @staticmethod
    def _current_physics_profile(payload: dict[str, Any]) -> dict[str, Any]:
        profiles = payload.get("profiles")
        if isinstance(profiles, dict) and isinstance(profiles.get(PHYSICS_PROFILE_NAME), dict):
            return profiles[PHYSICS_PROFILE_NAME]
        profile = payload.get(PHYSICS_PROFILE_NAME)
        if isinstance(profile, dict):
            return profile
        raise KeyError(f"profile '{PHYSICS_PROFILE_NAME}' not found")

    def _load_physics_params_into_fields(self, silent: bool = False) -> None:
        try:
            payload = json.loads(PHYSICS_PROFILE_PATH.read_text(encoding="utf-8"))
            profile = self._current_physics_profile(payload)
            for spec in PHYSICS_PARAM_SPECS:
                key = str(spec["key"])
                value = self._get_nested_value(profile, key)
                if value is None:
                    value = spec["default"]
                self.physics_param_vars[key].set(self._format_physics_value(value))
        except Exception as exc:
            self._set_physics_status(f"physics params: load failed: {exc}")
            return
        if not silent:
            self._set_physics_status("physics params: loaded")

    def _parse_physics_value(self, spec: dict[str, Any]) -> Any:
        key = str(spec["key"])
        raw = self.physics_param_vars[key].get().strip()
        lower = float(spec.get("lower", -math.inf))
        upper = float(spec.get("upper", math.inf))
        kind = str(spec.get("kind", "scalar"))
        vector_match = re.fullmatch(r"vector(\d+)", kind)
        if vector_match:
            expected_len = int(vector_match.group(1))
            pieces = [piece for piece in re.split(r"[,\s]+", raw) if piece]
            if len(pieces) != expected_len:
                raise ValueError(f"{spec['label']} must have exactly {expected_len} numbers")
            values = [clamp(float(piece), lower, upper) for piece in pieces]
            self.physics_param_vars[key].set(self._format_physics_value(values))
            return values
        value = clamp(float(raw), lower, upper)
        self.physics_param_vars[key].set(self._format_physics_value(value))
        return value

    def _apply_physics_params(self, restart: bool = False) -> None:
        try:
            payload = json.loads(PHYSICS_PROFILE_PATH.read_text(encoding="utf-8"))
            profile = self._current_physics_profile(payload)
            for spec in PHYSICS_PARAM_SPECS:
                self._set_nested_value(profile, str(spec["key"]), self._parse_physics_value(spec))
            stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_path = PHYSICS_PROFILE_PATH.with_name(f"{PHYSICS_PROFILE_PATH.name}.bak_gui_phys_{stamp}")
            shutil.copy2(PHYSICS_PROFILE_PATH, backup_path)
            PHYSICS_PROFILE_PATH.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
        except Exception as exc:
            self._set_physics_status(f"physics params: apply failed: {exc}")
            self.node.push_event(f"physics params apply failed: {exc}")
            return

        self._set_physics_status("physics params: applied; restart required for running sim")
        self.node.push_event(f"physics params applied: backup {backup_path.name}")
        if restart:
            self._restart_sim_stack_after_physics_apply()

    def _restart_sim_stack_after_physics_apply(self) -> None:
        self._set_sim_stack_status("sim: restarting with physics params")
        self.node.push_event("physics params restart requested")
        if self._sim_stack_running():
            self._terminate_sim_stack_process()

        def start_when_ports_release() -> None:
            if self._closed:
                return
            if self._sim_stack_running():
                self.root.after(500, start_when_ports_release)
                return
            self._start_sim_stack()

        self.root.after(1500, start_when_ports_release)

    def _toggle_autotune_panel(self) -> None:
        show = not self.autotune_visible.get()
        self.autotune_visible.set(show)
        if self.autotune_frame is None:
            return
        if show:
            self.autotune_frame.grid()
            self.autotune_toggle_button.config(text="Hide auto tune")
        else:
            self.autotune_frame.grid_remove()
            self.autotune_toggle_button.config(text="Show auto tune")

    def _autotune_running(self) -> bool:
        return self._autotune_process is not None and self._autotune_process.poll() is None

    def _set_autotune_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.autotune_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.autotune_status_var.set(text))
        except Exception:
            pass

    def _browse_autotune_bag(self) -> None:
        initial_dir = str(DEFAULT_AUTOTUNE_BAG.parent if DEFAULT_AUTOTUNE_BAG.parent.exists() else APP_ROOT)
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="Select real ROS2 .db3 bag",
            initialdir=initial_dir,
            filetypes=(("ROS2 sqlite bag", "*.db3"), ("All files", "*")),
        )
        if selected:
            self.autotune_bag_var.set(selected)
            self._set_autotune_status("autotune: bag selected")

    @staticmethod
    def _read_float_var(var: tk.StringVar, default: float, lower: float, upper: float) -> float:
        try:
            value = float(var.get())
        except ValueError:
            value = default
        value = clamp(value, lower, upper)
        var.set(f"{value:g}")
        return value

    @staticmethod
    def _read_int_var(var: tk.StringVar, default: int, lower: int, upper: int) -> int:
        try:
            value = int(float(var.get()))
        except ValueError:
            value = default
        value = int(clamp(value, lower, upper))
        var.set(str(value))
        return value

    def _show_autotune_monitor(self) -> None:
        if self.autotune_monitor_window is not None and self.autotune_monitor_window.winfo_exists():
            self.autotune_monitor_window.deiconify()
            self.autotune_monitor_window.lift()
            return

        win = tk.Toplevel(self.root)
        win.title("UUV Auto Tune Monitor")
        win.geometry("860x620")
        win.minsize(720, 480)
        win.protocol("WM_DELETE_WINDOW", self._hide_autotune_monitor)
        self.autotune_monitor_window = win

        outer = ttk.Frame(win, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, textvariable=self.autotune_monitor_status_var, anchor="w").grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Button(header, text="Open output", command=self._open_autotune_output).grid(row=0, column=1, padx=(8, 0))

        progress = ttk.Progressbar(
            outer,
            variable=self.autotune_monitor_progress_var,
            maximum=100.0,
            mode="determinate",
        )
        progress.grid(row=1, column=0, sticky="ew", pady=(6, 8))

        body = ttk.PanedWindow(outer, orient=tk.VERTICAL)
        body.grid(row=2, column=0, sticky="nsew")

        top = ttk.Frame(body)
        top.columnconfigure(0, weight=1)
        top.rowconfigure(0, weight=1)
        body.add(top, weight=2)

        self.autotune_tree = ttk.Treeview(
            top,
            columns=("status", "score", "details"),
            show="tree headings",
            height=7,
        )
        self.autotune_tree.heading("#0", text="candidate")
        self.autotune_tree.heading("status", text="status")
        self.autotune_tree.heading("score", text="score")
        self.autotune_tree.heading("details", text="details")
        self.autotune_tree.column("#0", width=170, stretch=False)
        self.autotune_tree.column("status", width=95, stretch=False)
        self.autotune_tree.column("score", width=90, stretch=False, anchor=tk.E)
        self.autotune_tree.column("details", width=420, stretch=True)
        tree_scroll = ttk.Scrollbar(top, orient=tk.VERTICAL, command=self.autotune_tree.yview)
        self.autotune_tree.configure(yscrollcommand=tree_scroll.set)
        self.autotune_tree.grid(row=0, column=0, sticky="nsew")
        tree_scroll.grid(row=0, column=1, sticky="ns")

        self.autotune_chart_canvas = tk.Canvas(top, height=170, bg="#101827", highlightthickness=0)
        self.autotune_chart_canvas.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        self.autotune_chart_canvas.bind("<Configure>", lambda _event: self._redraw_autotune_chart())

        log_frame = ttk.LabelFrame(body, text="Live Log", padding=4)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        body.add(log_frame, weight=3)

        self.autotune_log_text = tk.Text(
            log_frame,
            height=10,
            wrap="none",
            bg="#111111",
            fg="#e5e7eb",
            insertbackground="#e5e7eb",
            font=("Menlo", 11),
        )
        log_y = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.autotune_log_text.yview)
        log_x = ttk.Scrollbar(log_frame, orient=tk.HORIZONTAL, command=self.autotune_log_text.xview)
        self.autotune_log_text.configure(yscrollcommand=log_y.set, xscrollcommand=log_x.set)
        self.autotune_log_text.grid(row=0, column=0, sticky="nsew")
        log_y.grid(row=0, column=1, sticky="ns")
        log_x.grid(row=1, column=0, sticky="ew")

        for name in self._autotune_candidate_order:
            self._upsert_autotune_candidate_row(name)
        self._refresh_autotune_progress()
        self._redraw_autotune_chart()

    def _hide_autotune_monitor(self) -> None:
        if self.autotune_monitor_window is not None and self.autotune_monitor_window.winfo_exists():
            self.autotune_monitor_window.withdraw()

    def _reset_autotune_monitor(self, out_dir: Path) -> None:
        self._autotune_candidate_order = []
        self._autotune_candidate_rows = {}
        self._autotune_current_candidate = None
        self.autotune_monitor_progress_var.set(0.0)
        self.autotune_monitor_status_var.set(f"autotune monitor: starting -> {out_dir.name}")
        if self.autotune_tree is not None:
            for item in self.autotune_tree.get_children():
                self.autotune_tree.delete(item)
        if self.autotune_log_text is not None:
            self.autotune_log_text.configure(state=tk.NORMAL)
            self.autotune_log_text.delete("1.0", tk.END)
            self.autotune_log_text.configure(state=tk.DISABLED)
        self._redraw_autotune_chart()

    def _post_autotune_monitor_line(self, line: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self._handle_autotune_monitor_line(line)
            return
        try:
            self.root.after(0, lambda: self._handle_autotune_monitor_line(line))
        except Exception:
            pass

    def _append_autotune_monitor_log(self, line: str) -> None:
        if self.autotune_log_text is None:
            return
        try:
            self.autotune_log_text.configure(state=tk.NORMAL)
            self.autotune_log_text.insert(tk.END, line + "\n")
            line_count = int(self.autotune_log_text.index("end-1c").split(".")[0])
            if line_count > 1200:
                self.autotune_log_text.delete("1.0", "200.0")
            self.autotune_log_text.see(tk.END)
            self.autotune_log_text.configure(state=tk.DISABLED)
        except Exception:
            pass

    def _handle_autotune_monitor_line(self, line: str) -> None:
        self._append_autotune_monitor_log(line)

        candidates_match = re.match(r"^\[autotune\]\s+candidates=(.+)$", line)
        if candidates_match:
            names = [part.strip() for part in candidates_match.group(1).split(",") if part.strip()]
            self._autotune_candidate_order = names
            for name in names:
                row = self._autotune_candidate_rows.setdefault(
                    name,
                    {"status": "queued", "score": math.nan, "details": ""},
                )
                row.setdefault("status", "queued")
                self._upsert_autotune_candidate_row(name)
            self.autotune_monitor_status_var.set(f"autotune monitor: queued {len(names)} candidates")
            self._refresh_autotune_progress()
            self._redraw_autotune_chart()
            return

        candidate_match = re.match(r"^\[autotune\]\s+candidate\s+([^:]+):\s+(.+)$", line)
        if candidate_match:
            name = candidate_match.group(1).strip()
            detail = candidate_match.group(2).strip()
            if detail == "start":
                self._autotune_current_candidate = name
                self._set_autotune_candidate(name, "running", math.nan, "running closed-loop replay")
                self.autotune_monitor_status_var.set(f"autotune monitor: running {name}")
            elif detail.startswith("score="):
                try:
                    score = float(detail.split("=", 1)[1])
                except ValueError:
                    score = math.nan
                self._set_autotune_candidate(name, "ok", score, "comparison finished")
                self.autotune_monitor_status_var.set(f"autotune monitor: {name} score={score:.4f}")
            elif detail.startswith("failed"):
                self._set_autotune_candidate(name, "failed", math.inf, detail)
                self.autotune_monitor_status_var.set(f"autotune monitor: {name} failed")
            else:
                self._set_autotune_candidate(name, "running", math.nan, detail)
            self._refresh_autotune_progress()
            self._redraw_autotune_chart()
            return

        best_match = re.match(r"^\[autotune\]\s+best=([^\s]+)\s+score=([0-9.eE+-]+)", line)
        if best_match:
            name = best_match.group(1)
            score = float(best_match.group(2))
            self.autotune_monitor_status_var.set(f"autotune monitor: best {name} score={score:.4f}")
            self._set_autotune_candidate(name, "best", score, "selected best candidate")
            self._refresh_autotune_progress()
            self._redraw_autotune_chart()
            return

        if line.startswith("[autotune] out="):
            self.autotune_monitor_status_var.set(f"autotune monitor: output {line.split('=', 1)[1]}")
            return

        if self._autotune_current_candidate and line.startswith("["):
            short_line = line if len(line) <= 120 else f"{line[:117]}..."
            self.autotune_monitor_status_var.set(
                f"autotune monitor: {self._autotune_current_candidate} | {short_line}"
            )

    def _set_autotune_candidate(self, name: str, status: str, score: float, details: str) -> None:
        if name not in self._autotune_candidate_order:
            self._autotune_candidate_order.append(name)
        row = self._autotune_candidate_rows.setdefault(
            name,
            {"status": "queued", "score": math.nan, "details": ""},
        )
        row["status"] = status
        if math.isfinite(score) or score == math.inf:
            row["score"] = score
        row["details"] = details
        self._upsert_autotune_candidate_row(name)

    def _upsert_autotune_candidate_row(self, name: str) -> None:
        if self.autotune_tree is None:
            return
        row = self._autotune_candidate_rows.get(name, {})
        status = str(row.get("status", "queued"))
        score = row.get("score", math.nan)
        score_text = "n/a"
        if isinstance(score, (int, float)):
            if math.isfinite(float(score)):
                score_text = f"{float(score):.4f}"
            elif float(score) == math.inf:
                score_text = "failed"
        details = str(row.get("details", ""))
        if self.autotune_tree.exists(name):
            self.autotune_tree.item(name, text=name, values=(status, score_text, details))
        else:
            self.autotune_tree.insert("", tk.END, iid=name, text=name, values=(status, score_text, details))

    def _refresh_autotune_progress(self) -> None:
        total = len(self._autotune_candidate_order)
        completed = 0
        for row in self._autotune_candidate_rows.values():
            if row.get("status") in {"ok", "failed", "best"}:
                completed += 1
        value = 0.0 if total <= 0 else 100.0 * completed / total
        self.autotune_monitor_progress_var.set(value)

    def _redraw_autotune_chart(self) -> None:
        canvas = self.autotune_chart_canvas
        if canvas is None:
            return
        width = max(canvas.winfo_width(), 500)
        height = max(canvas.winfo_height(), 160)
        canvas.delete("all")
        canvas.create_rectangle(0, 0, width, height, fill="#101827", outline="")
        canvas.create_text(
            12,
            10,
            anchor="nw",
            fill="#d1d5db",
            font=("Menlo", 11, "bold"),
            text="Auto tune score by candidate (lower is better)",
        )
        names = self._autotune_candidate_order
        if not names:
            canvas.create_text(
                width / 2,
                height / 2,
                fill="#9ca3af",
                font=("Menlo", 12),
                text="waiting for candidate list...",
            )
            return

        finite_scores = [
            float(self._autotune_candidate_rows.get(name, {}).get("score"))
            for name in names
            if isinstance(self._autotune_candidate_rows.get(name, {}).get("score"), (int, float))
            and math.isfinite(float(self._autotune_candidate_rows.get(name, {}).get("score")))
        ]
        max_score = max(finite_scores) if finite_scores else 1.0
        chart_top = 34
        chart_bottom = height - 34
        chart_left = 34
        chart_right = width - 16
        bar_gap = 8
        bar_width = max(18, (chart_right - chart_left - bar_gap * max(len(names) - 1, 0)) / max(len(names), 1))

        canvas.create_line(chart_left, chart_bottom, chart_right, chart_bottom, fill="#334155")
        for idx, name in enumerate(names):
            row = self._autotune_candidate_rows.get(name, {})
            status = row.get("status", "queued")
            score = row.get("score", math.nan)
            x0 = chart_left + idx * (bar_width + bar_gap)
            x1 = min(x0 + bar_width, chart_right)
            color = "#475569"
            if status == "running":
                color = "#38bdf8"
            elif status == "ok":
                color = "#22c55e"
            elif status == "best":
                color = "#facc15"
            elif status == "failed":
                color = "#ef4444"

            if isinstance(score, (int, float)) and math.isfinite(float(score)):
                frac = clamp(float(score) / max(max_score, 1.0e-9), 0.02, 1.0)
                y0 = chart_bottom - frac * (chart_bottom - chart_top)
                canvas.create_rectangle(x0, y0, x1, chart_bottom, fill=color, outline="")
                canvas.create_text(
                    (x0 + x1) / 2,
                    max(chart_top + 8, y0 - 8),
                    fill="#e5e7eb",
                    font=("Menlo", 9),
                    text=f"{float(score):.2f}",
                )
            else:
                y0 = chart_bottom - 5
                canvas.create_rectangle(x0, y0, x1, chart_bottom, fill=color, outline="")

            label = name if len(name) <= 14 else f"{name[:12]}.."
            canvas.create_text((x0 + x1) / 2, height - 18, fill="#cbd5e1", font=("Menlo", 9), text=label)

    def _finish_autotune_monitor(self, rc: int, last_line: str) -> None:
        if rc == 0:
            self.autotune_monitor_status_var.set("autotune monitor: finished")
        elif rc == 130:
            self.autotune_monitor_status_var.set("autotune monitor: stopped")
        else:
            text = last_line if last_line else f"rc={rc}"
            if len(text) > 140:
                text = f"{text[:137]}..."
            self.autotune_monitor_status_var.set(f"autotune monitor: failed | {text}")

    def _start_autotune(self) -> None:
        if self._autotune_running():
            self._set_autotune_status("autotune: already running")
            return
        if self._sim_stack_running():
            self._set_autotune_status("autotune: stopping sim stack first")
            self._set_sim_stack_status("sim: stopping for autotune")
            self.node.push_event("autotune requested: stopping running sim stack")
            self._terminate_sim_stack_process()
        if not AUTOTUNE_SCRIPT.exists():
            self._set_autotune_status(f"autotune script missing: {AUTOTUNE_SCRIPT}")
            return

        bag_path = Path(self.autotune_bag_var.get()).expanduser()
        if not bag_path.exists():
            self._set_autotune_status(f"autotune: bag path missing: {bag_path}")
            return

        if self._rc_replay_running():
            self._stop_rc_replay()
        self.rc_override_enabled.set(False)
        self.node.publish_rc_release()

        start_s = self._read_float_var(self.autotune_start_var, 60.0, 0.0, 10000.0)
        duration_s = self._read_float_var(self.autotune_duration_var, 120.0, 10.0, 600.0)
        max_candidates = self._read_int_var(self.autotune_candidates_var, 15, 1, 30)
        servo_scale = self._read_float_var(self.autotune_servo_scale_var, 0.58, 0.1, 2.0)
        tune_mode = self.autotune_mode_var.get().strip() or "plant-rc-out"
        candidate_set = self.autotune_candidate_set_var.get().strip() or "ellipsoid5"
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = APP_ROOT / "document" / "docsource" / f"gui_autotune_{stamp}"
        self.autotune_out_dir_var.set(str(out_dir))
        self._show_autotune_monitor()
        self._reset_autotune_monitor(out_dir)

        autotune_python = resolve_autotune_python()
        cmd = [
            autotune_python,
            str(AUTOTUNE_SCRIPT),
            "--bag",
            str(bag_path),
            "--out-root",
            str(out_dir),
            "--start-offset-s",
            f"{start_s:g}",
            "--duration-s",
            f"{duration_s:g}",
            "--sitl-servo-scale",
            f"{servo_scale:g}",
            "--max-candidates",
            str(max_candidates),
            "--tune-mode",
            tune_mode,
            "--candidate-set",
            candidate_set,
        ]
        if self.autotune_apply_best_var.get():
            cmd.append("--apply-best")
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=str(APP_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
                start_new_session=True,
            )
        except Exception as exc:
            self._set_autotune_status(f"autotune start failed: {exc}")
            self.autotune_monitor_status_var.set(f"autotune monitor: start failed | {exc}")
            return

        self._autotune_process = proc
        self._set_autotune_status(f"autotune: running -> {out_dir.name}")
        self.node.push_event(f"autotune start: {out_dir.name}")
        self._autotune_thread = threading.Thread(
            target=self._read_autotune_output,
            args=(proc,),
            daemon=True,
        )
        self._autotune_thread.start()

    def _read_autotune_output(self, proc: subprocess.Popen[str]) -> None:
        last_line = ""
        try:
            assert proc.stdout is not None
            for raw_line in proc.stdout:
                line = raw_line.strip()
                if not line:
                    continue
                last_line = line
                self._post_autotune_monitor_line(line)
                if line.startswith("[autotune]"):
                    short_line = line if len(line) <= 150 else f"{line[:147]}..."
                    self.node.push_event(short_line)
                    self._set_autotune_status(f"autotune: {short_line.removeprefix('[autotune] ').strip()}")
            rc = proc.wait()
        except Exception as exc:
            rc = -1
            last_line = f"reader failed: {exc}"

        def finish() -> None:
            if self._autotune_process is proc:
                self._autotune_process = None
            if rc == 0:
                self.autotune_status_var.set("autotune: finished")
                self.node.push_event("autotune finished")
            elif rc == 130:
                self.autotune_status_var.set("autotune: stopped")
                self.node.push_event("autotune stopped")
            else:
                text = last_line if last_line else f"rc={rc}"
                self.autotune_status_var.set(f"autotune failed: {text}")
                self.node.push_event(f"autotune failed: {text}")
            self._finish_autotune_monitor(rc, last_line)

        try:
            self.root.after(0, finish)
        except Exception:
            pass

    def _stop_autotune(self) -> None:
        proc = self._autotune_process
        if proc is None or proc.poll() is not None:
            self._set_autotune_status("autotune: not running")
            return
        try:
            proc.terminate()
            self._set_autotune_status("autotune: stopping")
            self.node.push_event("autotune stop requested")
        except Exception as exc:
            self._set_autotune_status(f"autotune stop failed: {exc}")

    def _open_autotune_output(self) -> None:
        path_text = self.autotune_out_dir_var.get()
        if not path_text:
            self._set_autotune_status("autotune: no output yet")
            return
        path = Path(path_text).expanduser()
        if not path.exists():
            self._set_autotune_status(f"autotune output missing: {path}")
            return
        opener = "open" if sys.platform == "darwin" else "xdg-open"
        try:
            subprocess.Popen([opener, str(path)])
        except Exception as exc:
            self._set_autotune_status(f"open output failed: {exc}")

    def _on_rc_override_toggle(self) -> None:
        if self.rc_override_enabled.get() and self._rc_replay_running():
            self._stop_rc_replay()
        if self.rc_override_enabled.get():
            pass
        else:
            self.node.publish_rc_release()

    def _draw_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        canvas = self.attitude_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        height = max(canvas.winfo_height(), 100)
        cx = width / 2.0
        cy = height / 2.0

        pitch_offset = clamp(pitch_deg, -45.0, 45.0) * 2.2
        roll_rad = math.radians(roll_deg)
        extent = max(width, height) * 1.8
        half = extent / 2.0
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)

        def rot(x: float, y: float) -> tuple[float, float]:
            return (cx + x * cos_r - y * sin_r, cy + x * sin_r + y * cos_r)

        sky = [
            rot(-half, -half - pitch_offset),
            rot(half, -half - pitch_offset),
            rot(half, -pitch_offset),
            rot(-half, -pitch_offset),
        ]
        ground = [
            rot(-half, -pitch_offset),
            rot(half, -pitch_offset),
            rot(half, half - pitch_offset),
            rot(-half, half - pitch_offset),
        ]
        canvas.create_polygon(*sum(([x, y] for x, y in sky), []), fill="#1d4ed8", outline="")
        canvas.create_polygon(*sum(([x, y] for x, y in ground), []), fill="#854d0e", outline="")

        left = rot(-half, -pitch_offset)
        right = rot(half, -pitch_offset)
        canvas.create_line(left[0], left[1], right[0], right[1], fill="white", width=3)

        for step in range(-30, 35, 10):
            if step == 0:
                continue
            y_line = -pitch_offset - step * 2.2
            span = 60 if step % 20 == 0 else 30
            p1 = rot(-span, y_line)
            p2 = rot(span, y_line)
            canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill="#e2e8f0", width=2)

        canvas.create_line(cx - 70, cy, cx - 15, cy, fill="#f8fafc", width=4)
        canvas.create_line(cx + 15, cy, cx + 70, cy, fill="#f8fafc", width=4)
        canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, outline="#f8fafc", width=2)
        canvas.create_line(cx, cy - 18, cx, cy + 18, fill="#f8fafc", width=2)

        canvas.create_text(
            12,
            12,
            anchor="nw",
            fill="#f8fafc",
            font=("TkDefaultFont", 12, "bold"),
            text=f"ROLL {roll_deg:+05.1f}  PITCH {pitch_deg:+05.1f}  YAW {yaw_deg:+06.1f}",
        )

    def _draw_depth(self, depth_m: float, source: str) -> None:
        canvas = self.depth_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 140)
        height = max(canvas.winfo_height(), 82)
        pad = 12
        value_valid = math.isfinite(depth_m)
        display_depth = depth_m if value_valid else 0.0
        clamped_depth = clamp(display_depth, 0.0, MAX_DEPTH_DISPLAY_M)
        ratio = clamped_depth / max(MAX_DEPTH_DISPLAY_M, 1e-6)
        value_text = f"{depth_m:.2f} m" if value_valid else "n/a"
        source_text = source if len(source) <= 28 else f"{source[:25]}..."

        canvas.create_rectangle(0, 0, width, height, fill="#0f172a", outline="")
        canvas.create_text(pad, 11, anchor="nw", fill="#94a3b8", font=("TkDefaultFont", 9, "bold"), text="DEPTH")
        canvas.create_text(
            pad,
            31,
            anchor="w",
            fill="#e0f2fe" if value_valid else "#64748b",
            font=("TkDefaultFont", 20, "bold"),
            text=value_text,
        )
        canvas.create_text(width - pad, 15, anchor="ne", fill="#64748b", font=("TkDefaultFont", 8), text=source_text)

        bar_x0 = pad
        bar_x1 = width - pad
        bar_y0 = height - 26
        bar_y1 = height - 15
        canvas.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y1, fill="#1e293b", outline="#334155")
        if value_valid:
            canvas.create_rectangle(
                bar_x0 + 1,
                bar_y0 + 1,
                bar_x0 + 1 + (bar_x1 - bar_x0 - 2) * ratio,
                bar_y1 - 1,
                fill="#38bdf8",
                outline="",
            )
        canvas.create_text(bar_x0, height - 6, anchor="sw", fill="#94a3b8", font=("TkDefaultFont", 8), text="0")
        canvas.create_text(
            bar_x1,
            height - 6,
            anchor="se",
            fill="#94a3b8",
            font=("TkDefaultFont", 8),
            text=f"{MAX_DEPTH_DISPLAY_M:g} m",
        )

    def _update_events(self, events: Deque[str]) -> None:
        top = events[0] if events else ""
        if top == self._last_event_top:
            return
        self._last_event_top = top
        self.event_list.delete(0, tk.END)
        for item in events:
            self.event_list.insert(tk.END, item)

    def _read_control_commands(self) -> ControlCommands:
        rc_forward, rc_lateral, rc_heave, rc_yaw = gui_rc_to_override_axes(
            forward=self.rc_forward_var.get(),
            lateral=self.rc_lateral_var.get(),
            heave=self.rc_heave_var.get(),
            yaw=self.rc_yaw_var.get(),
        )
        return ControlCommands(
            velocity_forward=0.0,
            velocity_lateral=0.0,
            velocity_heave=0.0,
            velocity_yaw=0.0,
            rc_forward=rc_forward,
            rc_lateral=rc_lateral,
            rc_heave=rc_heave,
            rc_yaw=rc_yaw,
        )

    def _publish_active_controls(self, commands: ControlCommands) -> None:
        rc_active = self.rc_override_enabled.get()

        if rc_active:
            self.node.publish_rc_override(
                yaw=commands.rc_yaw,
                heave=commands.rc_heave,
                forward=commands.rc_forward,
                lateral=commands.rc_lateral,
            )
        elif self._rc_override_prev:
            self.node.publish_rc_release()
        self._rc_override_prev = rc_active

    def _update_rc_feedback_bars(self, channels: list[int]) -> None:
        for idx, channel in enumerate(channels[:RC_VISIBLE_CHANNEL_COUNT]):
            value = int(channel)
            self._rc_bars[idx]["value"] = clamp(value - 1100, 0, 800) if value > 0 else 0
            self._rc_labels[idx].config(text=str(value))

    def _update_ui(self) -> None:
        if self._closed or not self.root.winfo_exists():
            return
        snap = self.node.snapshot()
        self.node.probe_backend()
        now = time.monotonic()
        if now - self._last_vehicle_info_wall > 2.0:
            self.node.request_vehicle_info()
            self._last_vehicle_info_wall = now

        backend_label = self.node.backend_label()
        rc_mapping_summary = self.node.rc_mapping_summary()
        mode_display = snap.vehicle_mode or snap.mode
        state_text = (
            f"connected={snap.connected}  armed={snap.armed}  guided={snap.guided}  "
            f"manual_input={snap.manual_input}  backend={backend_label}"
        )
        self.status_var.set(state_text)
        self.mode_var.set(f"mode: {mode_display}  (raw={snap.mode}, id={snap.mode_id}, state={snap.system_status})")

        depth_summary = f"{snap.depth_m:.2f} m" if math.isfinite(snap.depth_m) else "n/a"
        vehicle_state = "connected" if snap.connected else "disconnected"
        arm_state = "armed" if snap.armed else "disarmed"
        self.vehicle_summary_var.set(
            f"{vehicle_state} | {arm_state} | {mode_display} | depth {depth_summary}"
        )

        batt_pct = snap.battery_percent * 100.0 if math.isfinite(snap.battery_percent) else math.nan
        batt_text = (
            f"battery: {snap.battery_voltage:.2f} V, {snap.battery_current:.2f} A, "
            f"{batt_pct:.0f}%"
            if math.isfinite(snap.battery_voltage)
            else "battery: n/a"
        )
        self.battery_var.set(batt_text)

        px, py, pz = snap.position_xyz
        self.pose_var.set(
            f"pose: x={px:+.2f}  y={py:+.2f}  z={pz:+.2f}"
            if math.isfinite(px)
            else "pose: n/a"
        )
        vx, vy, vz = snap.velocity_xyz
        self.vel_var.set(
            f"velocity: x={vx:+.2f}  y={vy:+.2f}  z={vz:+.2f}  src={snap.velocity_source}"
            if math.isfinite(vx)
            else f"velocity: n/a  src={snap.velocity_source}"
        )
        vel_summary = f"vel ({vx:+.2f}, {vy:+.2f}, {vz:+.2f}) m/s" if math.isfinite(vx) else "vel n/a"
        wx, wy, wz = snap.ang_vel_xyz
        self.imu_var.set(
            f"imu: roll={snap.roll_deg:+.1f}  pitch={snap.pitch_deg:+.1f}  yaw={snap.yaw_deg:+.1f}  "
            f"gyro=({wx:+.2f}, {wy:+.2f}, {wz:+.2f})"
        )
        self.motion_summary_var.set(
            f"{vel_summary} | rpy ({snap.roll_deg:+.1f}, {snap.pitch_deg:+.1f}, {snap.yaw_deg:+.1f})"
        )
        autopilot_text = snap.autopilot_name
        if not autopilot_text:
            if self.node.vehicle_info_supported():
                autopilot_text = "pending vehicle_info_get"
            else:
                autopilot_text = "vehicle_info_get unavailable"
        self.autopilot_var.set(
            f"autopilot: {autopilot_text}  rc-map={rc_mapping_summary}"
        )

        if math.isfinite(snap.depth_m):
            self.depth_target_var.set(f"depth: {snap.depth_m:.2f} m")
        else:
            self.depth_target_var.set("depth: n/a")
        self.depth_source_var.set(f"depth source: {snap.depth_source}")

        self.age_var.set(
            "age: "
            f"state={format_age(snap.state_age_s)}, "
            f"imu={format_age(snap.imu_age_s)}, "
            f"pose={format_age(snap.pose_age_s)}, "
            f"depth={format_age(snap.depth_age_s)}, "
            f"rc={format_age(snap.rc_age_s)}"
        )
        ping360_age = format_age(snap.ping360_age_s)
        self.ping360_summary_var.set(f"{snap.ping360_summary}  age={ping360_age}")

        commands = self._read_control_commands()
        if self.rc_override_enabled.get():
            control_mode = "RC override"
        elif self._rc_replay_running():
            control_mode = "RC replay"
        else:
            control_mode = "idle"
        self.control_summary_var.set(
            f"control: {control_mode}  details={'shown' if self.control_details_visible.get() else 'hidden'}"
        )
        self.control_var.set(
            f"rc setpoint: fwd={commands.rc_forward:+.2f}  lat={commands.rc_lateral:+.2f}  "
            f"heave={commands.rc_heave:+.2f}  yaw={commands.rc_yaw:+.2f}"
        )
        self.rc_override_var.set(
            "rc override: "
            f"{'on' if self.rc_override_enabled.get() else 'off'}  "
            f"{rc_mapping_summary}  "
            f"heave={axis_to_pwm(commands.rc_heave)}  yaw={axis_to_pwm(commands.rc_yaw)}  "
            f"forward={axis_to_pwm(commands.rc_forward)}  lateral={axis_to_pwm(commands.rc_lateral)}  "
            f"feedback={snap.rc_feedback_source}"
        )

        self._publish_active_controls(commands)

        self._draw_attitude(snap.roll_deg, snap.pitch_deg, snap.yaw_deg)
        self._draw_depth(snap.depth_m, snap.depth_source)
        self._update_events(snap.events)

        self._update_rc_feedback_bars(snap.rc_out)

        self._schedule_update()

    def _on_close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        self._rc_replay_stop_event.set()
        self._rc_replay_pause_event.clear()
        self._stop_ping360_view()
        self._terminate_sim_stack_process()
        if self._autotune_process is not None and self._autotune_process.poll() is None:
            try:
                self._autotune_process.terminate()
            except Exception:
                pass
        try:
            self.node.publish_rc_release()
        except Exception:
            pass
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UUV MAVROS telemetry and control GUI")
    parser.add_argument(
        "--namespace",
        default="/mavros",
        help="MAVROS namespace to use (default: /mavros)",
    )
    parser.add_argument(
        "--backend",
        choices=(BACKEND_AUTO, BACKEND_NONE, BACKEND_MAVROS, BACKEND_SIM_BRIDGE, "sim"),
        default=BACKEND_AUTO,
        help="Control/RC compatibility profile (default: auto)",
    )
    parser.add_argument(
        "--title",
        default="UUV Control GUI",
        help="GUI window title",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init(args=None)
    node = UuvGuiNode(namespace=args.namespace, backend=args.backend)
    app = UuvControlGui(node=node, title=args.title)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

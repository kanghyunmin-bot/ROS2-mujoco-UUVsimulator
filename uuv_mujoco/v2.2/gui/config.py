"""Configuration constants and paths for the MuJoCo UUV GUI."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

BACKEND_AUTO = "auto"
BACKEND_NONE = "none"
BACKEND_MAVROS = "mavros"
BACKEND_SIM_BRIDGE = "sim_bridge"
DEFAULT_AUTO_BACKEND = BACKEND_NONE
GUI_DIR = Path(__file__).resolve().parent
SIM_STACK_DIR = GUI_DIR.parent
UUV_MUJOCO_DIR = SIM_STACK_DIR.parent
PROJECT_ROOT = UUV_MUJOCO_DIR.parent
# APP_ROOT is kept as the workspace root because user-facing defaults, bag
# replay paths, rospkg, and document outputs live outside uuv_mujoco/v2.2.
APP_ROOT = PROJECT_ROOT

AXIS_MIN = -1.0
AXIS_MAX = 1.0
AXIS_DEADBAND = 0.03
RC_NEUTRAL_PWM = 1500
RC_PWM_SPAN = 400.0
RC_MESSAGE_CHANNEL_COUNT = 18
RC_FEEDBACK_CHANNEL_COUNT = 16
RC_VISIBLE_CHANNEL_COUNT = 8
PRIMARY_RC_CHANNEL_COUNT = 6
TELEMETRY_EVENT_LIMIT = 200
PILOT_CONTROL_MANUAL = "manual_control"
PILOT_CONTROL_RC_OVERRIDE = "rc_override"


def _pilot_control_mode() -> str:
    mode = os.environ.get("UUV_GUI_PILOT_CONTROL_MODE", PILOT_CONTROL_MANUAL).strip().lower()
    if mode in {PILOT_CONTROL_MANUAL, PILOT_CONTROL_RC_OVERRIDE}:
        return mode
    return PILOT_CONTROL_MANUAL


def _env_float_default(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


GUI_PILOT_CONTROL_MODE = _pilot_control_mode()
REAL_JS_GAIN_DEFAULT = _env_float_default("SITL_JS_GAIN_DEFAULT", 0.1)
REAL_JS_GAIN_MIN = _env_float_default("SITL_JS_GAIN_MIN", 0.25)
REAL_JS_GAIN_MAX = _env_float_default("SITL_JS_GAIN_MAX", 2.0)
REAL_JS_GAIN_STEPS = int(_env_float_default("SITL_JS_GAIN_STEPS", 4))
REAL_JS_THR_GAIN = _env_float_default("SITL_JS_THR_GAIN", 1.0)
REAL_RC3_MIN = int(_env_float_default("SITL_RC3_MIN", 1100))
REAL_RC3_MAX = int(_env_float_default("SITL_RC3_MAX", 1900))
REAL_RC3_TRIM = int(_env_float_default("SITL_RC3_TRIM", 1100))
REAL_RC3_DZ = int(_env_float_default("SITL_RC3_DZ", 30))
REAL_PILOT_SPEED_UP = _env_float_default("SITL_PILOT_SPEED_UP", 100)
REAL_PILOT_SPEED_DN = _env_float_default("SITL_PILOT_SPEED_DN", 0)


def _runtime_profile() -> str:
    profile = os.environ.get("UUV_RUNTIME_PROFILE", "balanced").strip().lower()
    if profile in {"low", "balanced", "high"}:
        return profile
    return "balanced"


def _profile_default_update_ms() -> int:
    return {
        "low": 200,
        "balanced": 100,
        "high": 50,
    }[_runtime_profile()]


def _env_int(name: str, default: int, min_value: int, max_value: int) -> int:
    try:
        value = int(os.environ.get(name, default))
    except (TypeError, ValueError):
        value = default
    return max(min_value, min(max_value, value))


UI_UPDATE_PERIOD_MS = _env_int("UUV_GUI_UPDATE_MS", _profile_default_update_ms(), 50, 500)
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
START_SIM_STACK_SCRIPT = SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh"
START_DOCKER_SIM_STACK_SCRIPT = SIM_STACK_DIR / "start_docker_sitl_mujoco_mj311.sh"
RESET_SIM_STACK_SCRIPT = SIM_STACK_DIR / "reset_uuv_sim.sh"
STOP_DOCKER_SITL_SCRIPT = SIM_STACK_DIR / "stop_docker_ardusub_sitl.sh"
PING360_RVIZ_CONFIG = SIM_STACK_DIR / "generated" / "rviz" / "ping360.rviz"
ROS_WORKSPACE_DIR = APP_ROOT / "rospkg"
ROS_PACKAGE_DIR = ROS_WORKSPACE_DIR / "kmu26_auv"
PING360_MSG_PACKAGE_DIR = ROS_WORKSPACE_DIR / "ping360_sonar_msgs"
ROS_PACKAGE_NAME = "hit25_auv_ros2"
ROS_PACKAGE_LAUNCH_FILE = "rov_start.launch.py"
ROS_PACKAGE_RVIZ_CONFIG = ROS_PACKAGE_DIR / "rviz" / "rov.rviz"
ROS2_RVIZ_COMPAT_CONFIG = SIM_STACK_DIR / "generated" / "rviz" / "rov_ros2_compat.rviz"
ROS_PACKAGE_DEFAULT_FCU_URL = "udp://:14551@127.0.0.1:14551"
# Local macOS fallback for MAVROS. On Ubuntu, install the normal GeographicLib
# dataset package/script and let MAVROS use its system path.
GEOGRAPHICLIB_DATA_DIR = APP_ROOT / ".geographiclib"
GEOGRAPHICLIB_GEOID_DIR = GEOGRAPHICLIB_DATA_DIR / "geoids"
PHYSICS_PROFILE_PATH = SIM_STACK_DIR / "config" / "sim_profiles.json"
PHYSICS_PROFILE_NAME = "current"
PHYSICS_PARAM_SPECS: tuple[dict[str, Any], ...] = (
    {
        "key": "buoyancy_scale",
        "label": "Buoyancy scale",
        "default": 1.0005,
        "description": "Net buoyancy multiplier. Small changes strongly affect depth trim.",
    },
    {
        "key": "surface_heave_damping",
        "label": "Surface heave damping",
        "default": 13.2,
        "description": "Extra vertical damping near the water surface.",
    },
    {
        "key": "heave_damping_scale",
        "label": "Heave damping scale",
        "default": 6.0,
        "description": "Fully-submerged vertical damping multiplier.",
    },
    {
        "key": "buoyancy_slope_scale",
        "label": "Buoyancy slope scale",
        "default": 1.2,
        "description": "Waterline buoyancy transition sharpness. High values can make depth response stiff.",
    },
    {
        "key": "cob_torque_scale",
        "label": "CoB torque scale",
        "default": 0.35,
        "description": "Restoring moment gain from CB-CG offset. Main roll/pitch stability knob.",
    },
    {
        "key": "cob_z_offset",
        "label": "CoB z offset",
        "default": 0.012,
        "description": "Vertical CB-CG offset. Higher usually increases roll/pitch restoring.",
    },
    {
        "key": "cob_x_offset",
        "label": "CoB x offset",
        "default": 0.009,
        "description": "Forward CB offset. Tunes pitch trim under buoyancy.",
    },
    {
        "key": "buoyancy_point_blend",
        "label": "Buoyancy point blend",
        "default": 1.0,
        "description": "0=center buoyancy, 1=distributed buoyancy points. Affects roll/pitch torque.",
    },
    {
        "key": "thruster_force_max",
        "label": "Thruster force max",
        "default": 21.0,
        "description": "Per-thruster force limit used by the disabled-performance simple model.",
    },
    {
        "key": "linear_drag",
        "label": "Legacy linear drag",
        "default": 0.93,
        "description": "Legacy translational drag scalar when non-ellipsoid hydro terms are active.",
    },
    {
        "key": "angular_drag",
        "label": "Legacy angular drag",
        "default": 0.72,
        "description": "Legacy angular drag scalar when non-ellipsoid hydro terms are active.",
    },
    {
        "key": "ellipsoid_model.effective_cd_linear",
        "label": "CD linear x y z",
        "kind": "vector3",
        "default": [0.06, 0.057, 0.087],
        "description": "Legacy/custom ellipsoid translational drag. In current mode use MuJoCo fluidcoef scale.",
    },
    {
        "key": "ellipsoid_model.effective_cd_angular",
        "label": "CD angular r p y",
        "kind": "vector3",
        "default": [1.62, 0.78, 0.003],
        "description": "Legacy/custom ellipsoid rotational drag. In current mode use MuJoCo fluidcoef scale.",
    },
    {
        "key": "mujoco_fluidcoef_scale",
        "label": "MuJoCo fluidcoef scale",
        "kind": "vector5",
        "default": [1.0, 1.0, 1.0, 1.0, 1.0],
        "description": "Active current-mode geom fluidcoef scale: blunt, slender, angular, Kutta, Magnus.",
    },
    {
        "key": "linear_damping_linear",
        "label": "6DOF linear damping xyz",
        "kind": "vector3",
        "default": [1.10, 1.32, 1.54],
        "description": "Custom/legacy 6-DOF translational linear damping. Used when fluid model is legacy.",
    },
    {
        "key": "linear_damping_angular",
        "label": "6DOF linear damping rpy",
        "kind": "vector3",
        "default": [0.32, 0.36, 0.28],
        "description": "Custom/legacy 6-DOF rotational linear damping. Used when fluid model is legacy.",
    },
    {
        "key": "quadratic_damping_linear",
        "label": "6DOF quad damping xyz",
        "kind": "vector3",
        "default": [1.40, 2.00, 2.40],
        "description": "Custom/legacy 6-DOF translational |v|v damping. Used when fluid model is legacy.",
    },
    {
        "key": "quadratic_damping_angular",
        "label": "6DOF quad damping rpy",
        "kind": "vector3",
        "default": [0.10, 0.12, 0.08],
        "description": "Custom/legacy 6-DOF rotational |omega|omega damping. Used when fluid model is legacy.",
    },
    {
        "key": "ellipsoid_model.added_mass_scale_linear",
        "label": "Added mass x y z",
        "kind": "vector3",
        "default": [0.45, 0.28, 0.65],
        "description": "Legacy/custom translational added-mass scale. Inactive in current MuJoCo-fluid mode.",
    },
    {
        "key": "ellipsoid_model.added_mass_scale_angular",
        "label": "Added mass r p y",
        "kind": "vector3",
        "default": [1.0, 0.92, 1.02],
        "description": "Legacy/custom rotational added-mass scale. Inactive in current MuJoCo-fluid mode.",
    },
    {
        "key": "ellipsoid_model.linear_damping_ratio_linear",
        "label": "Low-speed linear damping",
        "default": 2.0,
        "description": "Legacy/custom low-speed translational damping. Inactive in current MuJoCo-fluid mode.",
    },
    {
        "key": "ellipsoid_model.linear_damping_ratio_angular",
        "label": "Low-speed angular damping",
        "default": 4.2,
        "description": "Legacy/custom low-speed angular damping. Inactive in current MuJoCo-fluid mode.",
    },
    {
        "key": "ellipsoid_model.reference_speed_linear",
        "label": "Ref speed linear",
        "default": 0.3,
        "description": "Legacy/custom reference speed for derived linear damping.",
    },
    {
        "key": "ellipsoid_model.reference_speed_angular",
        "label": "Ref speed angular",
        "default": 0.6,
        "description": "Legacy/custom reference angular speed for derived angular damping.",
    },
    {
        "key": "air_linear_drag",
        "label": "Air linear drag",
        "default": 0.03,
        "description": "Above-water translational damping fallback.",
    },
    {
        "key": "air_angular_drag",
        "label": "Air angular drag",
        "default": 0.03,
        "description": "Above-water rotational damping fallback.",
    },
    {
        "key": "current_world",
        "label": "Current world xyz",
        "kind": "vector3",
        "default": [0.0, 0.0, 0.0],
        "description": "Water-current velocity in world frame.",
    },
    {
        "key": "body_inertia_scale_xyz",
        "label": "Body inertia x y z",
        "kind": "vector3",
        "default": [1.0, 1.0, 1.0],
        "description": "Body rotational inertia scale. Affects roll/pitch/yaw acceleration.",
    },
    {
        "key": "yaw_torque_scale",
        "label": "Yaw torque scale",
        "default": 1.0,
        "description": "Additional yaw torque multiplier. Keep near 1 unless yaw rate is off.",
    },
)

CURRENT_MODE_INACTIVE_PHYSICS_KEYS: dict[str, str] = {
    "thruster_force_max": "inactive while the T200 performance curve is active",
    "linear_drag": "legacy/custom hydrodynamics only",
    "angular_drag": "legacy/custom hydrodynamics only",
    "ellipsoid_model.effective_cd_linear": "legacy/custom hydrodynamics only",
    "ellipsoid_model.effective_cd_angular": "legacy/custom hydrodynamics only",
    "linear_damping_linear": "legacy/custom hydrodynamics only",
    "linear_damping_angular": "legacy/custom hydrodynamics only",
    "quadratic_damping_linear": "legacy/custom hydrodynamics only",
    "quadratic_damping_angular": "legacy/custom hydrodynamics only",
    "ellipsoid_model.added_mass_scale_linear": "legacy/custom hydrodynamics only",
    "ellipsoid_model.added_mass_scale_angular": "legacy/custom hydrodynamics only",
    "ellipsoid_model.linear_damping_ratio_linear": "legacy/custom hydrodynamics only",
    "ellipsoid_model.linear_damping_ratio_angular": "legacy/custom hydrodynamics only",
    "ellipsoid_model.reference_speed_linear": "legacy/custom hydrodynamics only",
    "ellipsoid_model.reference_speed_angular": "legacy/custom hydrodynamics only",
}


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

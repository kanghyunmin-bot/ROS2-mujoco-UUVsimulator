"""Configuration constants and paths for the MuJoCo UUV GUI."""

from __future__ import annotations

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
START_SIM_STACK_SCRIPT = SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh"
RESET_SIM_STACK_SCRIPT = SIM_STACK_DIR / "reset_uuv_sim.sh"
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

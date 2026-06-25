"""Path constants for the MuJoCo UUV GUI."""

from __future__ import annotations

from pathlib import Path


GUI_DIR = Path(__file__).resolve().parent
SIM_STACK_DIR = GUI_DIR.parent
UUV_MUJOCO_DIR = SIM_STACK_DIR.parent
PROJECT_ROOT = UUV_MUJOCO_DIR.parent
# APP_ROOT is kept as the workspace root because user-facing defaults, bag
# replay paths, rospkg, and document outputs live outside the active runtime.
APP_ROOT = PROJECT_ROOT

RC_REPLAY_TOPIC = "/mavros/rc/override"
DEFAULT_RC_REPLAY_BAG = APP_ROOT / (
    "real_robot_ros_bag/extracted_2026_04_01/"
    "bag_2026-04-01_20-08-11_rc_override_no_invert"
)
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
COURSE_SCENE_PATH = SIM_STACK_DIR / "scenes" / "tank_current_scene.xml"


__all__ = [
    "APP_ROOT",
    "COURSE_SCENE_PATH",
    "DEFAULT_RC_REPLAY_BAG",
    "GEOGRAPHICLIB_DATA_DIR",
    "GEOGRAPHICLIB_GEOID_DIR",
    "GUI_DIR",
    "PHYSICS_PROFILE_PATH",
    "PING360_MSG_PACKAGE_DIR",
    "PING360_RVIZ_CONFIG",
    "PROJECT_ROOT",
    "RC_REPLAY_TOPIC",
    "RESET_SIM_STACK_SCRIPT",
    "ROS2_RVIZ_COMPAT_CONFIG",
    "ROS_PACKAGE_DEFAULT_FCU_URL",
    "ROS_PACKAGE_DIR",
    "ROS_PACKAGE_LAUNCH_FILE",
    "ROS_PACKAGE_NAME",
    "ROS_PACKAGE_RVIZ_CONFIG",
    "ROS_WORKSPACE_DIR",
    "SIM_STACK_DIR",
    "START_DOCKER_SIM_STACK_SCRIPT",
    "START_SIM_STACK_SCRIPT",
    "STOP_DOCKER_SITL_SCRIPT",
    "UUV_MUJOCO_DIR",
]

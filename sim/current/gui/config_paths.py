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
RESET_SIM_STACK_SCRIPT = SIM_STACK_DIR / "reset_uuv_sim.sh"
PING360_RVIZ_CONFIG = SIM_STACK_DIR / "generated" / "rviz" / "ping360.rviz"
ROS_WORKSPACE_DIR = APP_ROOT / "rospkg"
ROS_SOURCE_DIR = ROS_WORKSPACE_DIR / "src"
if not ROS_SOURCE_DIR.is_dir():
    # Compatibility with dist2/dist3 developer workspaces. New installs always
    # use the standard colcon rospkg/src layout.
    ROS_SOURCE_DIR = ROS_WORKSPACE_DIR
ROS_PACKAGE_DIR = ROS_SOURCE_DIR / "kmu26_auv"
PING360_MSG_PACKAGE_DIR = ROS_SOURCE_DIR / "ping360_sonar_msgs"
# The upstream package was renamed to ``auv``.  A stale
# ``install/hit25_auv_ros2`` directory can survive an incremental colcon build
# and hide this mismatch, so always launch the package that exists in src.
ROS_PACKAGE_NAME = "auv"
ROS_PACKAGE_LAUNCH_FILE = "rov_start.launch.py"
ROS_PACKAGE_RVIZ_CONFIG = ROS_PACKAGE_DIR / "rviz" / "rov.rviz"
ROS2_RVIZ_COMPAT_CONFIG = SIM_STACK_DIR / "generated" / "rviz" / "rov_ros2_compat.rviz"
# Listener-only MAVROS endpoint.  Leaving the remote side empty lets libmavconn
# learn MAVProxy's ephemeral UDP source port from the first FCU datagram.  A
# configured remote of 0.0.0.0:0 is not a wildcard: an early MAVROS transmit
# fails with EINVAL and closes the channel before the FCU heartbeat arrives.
ROS_PACKAGE_DEFAULT_FCU_URL = "udp://0.0.0.0:14551@"
# Local macOS fallback for MAVROS. On Ubuntu, install the normal GeographicLib
# dataset package/script and let MAVROS use its system path.
GEOGRAPHICLIB_DATA_DIR = APP_ROOT / ".geographiclib"
GEOGRAPHICLIB_GEOID_DIR = GEOGRAPHICLIB_DATA_DIR / "geoids"
PHYSICS_PROFILE_PATH = SIM_STACK_DIR / "config" / "sim_profiles.json"
COURSE_SCENE_PATH = SIM_STACK_DIR / "scenes" / "tank_current_scene.xml"
COURSE_LAYOUT_CONFIG_PATH = SIM_STACK_DIR / "config" / "course_layout.json"
TEST_TANK_SCENE_PATH = SIM_STACK_DIR / "generated" / "test_tank_gui_scene.xml"


__all__ = [
    "APP_ROOT",
    "COURSE_LAYOUT_CONFIG_PATH",
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
    "ROS_SOURCE_DIR",
    "ROS_WORKSPACE_DIR",
    "SIM_STACK_DIR",
    "START_SIM_STACK_SCRIPT",
    "TEST_TANK_SCENE_PATH",
    "UUV_MUJOCO_DIR",
]

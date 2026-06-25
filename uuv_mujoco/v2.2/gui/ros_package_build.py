"""ROS package build command helpers for the GUI."""

from __future__ import annotations

import shlex
from typing import Any

from .config import PING360_MSG_PACKAGE_DIR, ROS_PACKAGE_DIR
from .ros_tools import ros_bash_command


def ros_package_build_base_paths() -> list[str]:
    base_paths: list[str] = []
    if PING360_MSG_PACKAGE_DIR.exists():
        base_paths.append("ping360_sonar_msgs")
    base_paths.append("kmu26_auv")
    return base_paths


def ros_package_build_command() -> str:
    return (
        "colcon build --base-paths "
        + " ".join(shlex.quote(path) for path in ros_package_build_base_paths())
        + " --symlink-install --cmake-clean-cache"
    )


def start_ros_pkg_build(owner: Any) -> None:
    if owner._ros_build_running():
        owner._set_ros_pkg_status("ros2 build: already running")
        return
    if not ROS_PACKAGE_DIR.exists():
        owner._set_ros_pkg_status(f"ros2 build: package missing: {ROS_PACKAGE_DIR}")
        return
    owner._start_logged_ros_process(
        cmd=ros_bash_command(ros_package_build_command(), include_workspace=False),
        label="ros2 build",
        log_prefix="gui_ros2_build",
        attr_name="_ros_build_process",
        status_callback=owner._set_ros_pkg_status,
    )


__all__ = ["ros_package_build_base_paths", "ros_package_build_command", "start_ros_pkg_build"]

"""ROS package build command helpers for the GUI."""

from __future__ import annotations

import shlex
from typing import Any

from .config import APP_ROOT, PING360_MSG_PACKAGE_DIR, ROS_PACKAGE_DIR, ROS_WORKSPACE_DIR
from .ros_tools import ros_bash_command


def ros_package_build_base_paths() -> list[str]:
    base_paths: list[str] = []
    if PING360_MSG_PACKAGE_DIR.exists():
        base_paths.append(str(PING360_MSG_PACKAGE_DIR.relative_to(ROS_WORKSPACE_DIR)))
    base_paths.append(str(ROS_PACKAGE_DIR.relative_to(ROS_WORKSPACE_DIR)))
    return base_paths


def ros_package_build_command() -> str:
    # colcon's sequential executor limits packages, but colcon-cmake otherwise
    # still expands every CMake package to `-j<all CPUs>`.  That was enough to
    # exhaust this NUC/laptop while the Eigen-heavy pinger controller and the
    # simulator were open.  MAKEFLAGS is the documented colcon-cmake override
    # for its inner make invocation; keep ccache scratch under the writable
    # workspace instead of /run/user so GUI builds cannot hang on that path.
    ccache_dir = APP_ROOT / ".cache" / "ccache"
    ccache_temp_dir = APP_ROOT / ".cache" / "ccache-tmp"
    ccache_dir.mkdir(parents=True, exist_ok=True)
    ccache_temp_dir.mkdir(parents=True, exist_ok=True)
    return (
        "env MAKEFLAGS=-j1 "
        + "CCACHE_DIR=" + shlex.quote(str(ccache_dir)) + " "
        + "CCACHE_TEMPDIR=" + shlex.quote(str(ccache_temp_dir)) + " "
        + "colcon build --executor sequential --base-paths "
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

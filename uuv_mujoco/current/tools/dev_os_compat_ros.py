"""ROS environment checks for development OS compatibility."""

from __future__ import annotations

import os
import shutil

from dev_os_compat_common import CheckResult


def check_ros2_env(results: list[CheckResult]) -> None:
    ros_distro = os.environ.get("ROS_DISTRO", "")
    ros2 = shutil.which("ros2")
    if ros_distro or ros2:
        results.append(CheckResult("ros2_env", "pass", f"ROS_DISTRO={ros_distro!r} ros2={ros2 or 'not on PATH'}"))
    else:
        results.append(
            CheckResult(
                "ros2_env",
                "warn",
                "ROS2 not sourced in current shell; GUI launcher may source its configured environment",
            )
        )

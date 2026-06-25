"""MAVROS package stack start/stop helpers for the GUI."""

from __future__ import annotations

import shlex
from typing import Any

from .config import ROS_PACKAGE_DEFAULT_FCU_URL
from .ros_tools import ros_bash_command


def mavros_launch_command(fcu_url: str) -> str:
    launch_args = [
        "ros2",
        "launch",
        "mavros",
        "apm.launch",
        f"fcu_url:={fcu_url}",
    ]
    return " ".join(shlex.quote(part) for part in launch_args)


def stop_ros_pkg_stack(owner: Any) -> None:
    sim_was_running = owner._sim_stack_running()
    owner._terminate_process_group(owner._ros_pkg_process)
    owner._set_ros_pkg_status("mavros: stopping")
    owner._refresh_ros2_buttons()
    if sim_was_running:
        owner._restart_sim_stack_after_mavros_mode_change(
            want_mavros_running=False,
            reason="MAVROS OFF",
        )


def start_ros_pkg_stack(owner: Any) -> None:
    if owner._ros_build_running():
        owner._set_ros_pkg_status("mavros: wait for package build to finish")
        return
    fcu_url = owner.ros_pkg_fcu_url_var.get().strip() or ROS_PACKAGE_DEFAULT_FCU_URL
    owner.ros_pkg_fcu_url_var.set(fcu_url)
    owner._start_logged_ros_process(
        cmd=ros_bash_command(mavros_launch_command(fcu_url), include_workspace=False),
        label="mavros",
        log_prefix="gui_mavros_pkg",
        attr_name="_ros_pkg_process",
        status_callback=owner._set_ros_pkg_status,
    )
    if owner._sim_stack_running() and owner._ros_pkg_running():
        owner._restart_sim_stack_after_mavros_mode_change(
            want_mavros_running=True,
            reason="MAVROS ON",
        )


def toggle_ros_pkg_stack(owner: Any) -> None:
    if owner._ros_pkg_running():
        stop_ros_pkg_stack(owner)
        return
    start_ros_pkg_stack(owner)


__all__ = ["mavros_launch_command", "start_ros_pkg_stack", "stop_ros_pkg_stack", "toggle_ros_pkg_stack"]

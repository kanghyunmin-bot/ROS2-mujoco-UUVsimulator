"""MAVROS package stack start/stop helpers for the GUI."""

from __future__ import annotations

import shlex
from typing import Any

from .config import (
    ROS_PACKAGE_DEFAULT_FCU_URL,
    ROS_PACKAGE_LAUNCH_FILE,
    ROS_PACKAGE_NAME,
)
from .ros_tools import ros_bash_command


def mavros_launch_command(fcu_url: str) -> str:
    """Launch the complete real-vehicle ROS surface used by strict simulation.

    The Web GUI is already running, so the included real stack must not start a
    second GUI or desktop visualizers. The DVL uses the physical A50 driver
    against the MuJoCo TCP device emulator; simulator-owned camera and FCU
    sensor boundaries remain selected by ``use_sim_time``.
    """
    launch_args = [
        "ros2",
        "launch",
        ROS_PACKAGE_NAME,
        ROS_PACKAGE_LAUNCH_FILE,
        f"fcu_url:={fcu_url}",
        # This helper is only used by the GUI's MuJoCo compatibility stack.
        # Select the simulation MAVROS wrapper (timesync NONE) and keep the
        # DVL/depth/EKF graph on the same clock as the simulated sensors.
        "use_sim_time:=true",
        "use_dvl:=true",
        "dvl_ip:=127.0.0.1",
        "configure_dvl_acoustic_on_startup:=true",
        "request_dvl_config_on_startup:=true",
        "use_joy2mavros:=false",
        "use_battery_bridge:=false",
        "use_odom2mavros:=false",
        "publish_static_tf:=false",
        "use_localization:=true",
        "use_ekf:=true",
        # In simulation the Bar30 source is absolute water pressure.  Preserve
        # the real node defaults, but make this helper publish base_link depth
        # in the same surface-relative frame used by tank_max_depth_m.
        "surface_pressure_pa:=101640.0",
        "depth_zero_at_start:=false",
        "depth_offset_m:=0.03286",
        "use_web_gui:=false",
        "use_rviz:=false",
        "use_mission_rviz_visualizer:=false",
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
        cmd=ros_bash_command(mavros_launch_command(fcu_url), include_workspace=True),
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

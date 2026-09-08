"""Constructor implementation for the Ros2Bridge compatibility class."""

from __future__ import annotations

from typing import Any

from . import (
    ros2_bridge_init,
    ros2_bridge_runtime_setup,
    ros2_bridge_sensor_setup,
    ros2_sitl_transport_setup,
)
from .ros2_bridge_shutdown_guard import run_shutdown_step
from .ros2_bridge_shutdown_steps import (
    close_dvl_device_emulator,
    close_stereo_image_renderers,
    destroy_ros_node,
    remove_executor_node,
    shutdown_executor,
    shutdown_ros_context,
    shutdown_sitl_transport,
    stop_ros_spin_thread,
    stop_sitl_poll_thread,
)


def initialize_ros2_bridge(bridge: Any, init: dict[str, Any]) -> None:
    """Populate Ros2Bridge runtime state while preserving the public constructor."""

    try:
        ros2_bridge_runtime_setup.configure_bridge_runtime_state(
            bridge,
            model=init["model"],
            command_callback=init["command_callback"],
            cmd_limit=init["cmd_limit"],
            publish_images=init["publish_images"],
            image_width=init["image_width"],
            image_height=init["image_height"],
            sensor_hz=init["sensor_hz"],
            image_hz=init["image_hz"],
            camera_calib_left=init["camera_calib_left"],
            camera_calib_right=init["camera_calib_right"],
            enable_sitl=init["enable_sitl"],
            enable_ros=init["enable_ros"],
            enable_mavros_surface=init["enable_mavros_surface"],
            real_pkg_compat=init["real_pkg_compat"],
            unsafe_legacy_ground_truth_odometry_filtered=init.get(
                "unsafe_legacy_ground_truth_odometry_filtered",
                False,
            ),
            strict_sitl_sensor_transport=init.get(
                "strict_sitl_sensor_transport"
            ),
        )
        ros2_bridge_runtime_setup.configure_command_runtime_state(bridge)
        ros2_bridge_runtime_setup.configure_bridge_contracts(bridge)
        ros2_bridge_sensor_setup.configure_mujoco_sensor_runtime(
            bridge,
            enable_ping360=init["enable_ping360"],
            ping360_config_path=init["ping360_config_path"],
            ping360_overrides=init["ping360_overrides"],
        )

        bridge._sitl_transport = (
            ros2_sitl_transport_setup.create_sitl_transport_if_enabled(
                bridge,
                enable_sitl=bridge.enable_sitl,
                sitl_ip=init["sitl_ip"],
                sitl_port=int(init["sitl_port"]),
                sitl_send_port=int(init["sitl_send_port"]),
                sitl_mavlink_endpoint=init["sitl_mavlink_endpoint"],
                sitl_mavlink_servo_hz=float(init["sitl_mavlink_servo_hz"]),
                sitl_mavlink_target_sysid=int(init["sitl_mavlink_target_sysid"]),
                sitl_mavlink_target_compid=int(init["sitl_mavlink_target_compid"]),
                sitl_mavlink_source_sysid=int(init["sitl_mavlink_source_sysid"]),
                sitl_mavlink_source_compid=int(init["sitl_mavlink_source_compid"]),
            )
        )
        bridge._start_sitl_poll_thread()

        ros2_bridge_init.configure_ros_runtime_state(bridge)
        if bridge._enable_ros:
            bridge._init_ros()
    except Exception:
        _rollback_partial_initialization(bridge)
        raise


def _rollback_partial_initialization(bridge: Any) -> None:
    """Best-effort cleanup for every resource a partial constructor can own."""

    for step in (
        close_stereo_image_renderers,
        close_dvl_device_emulator,
        stop_ros_spin_thread,
        stop_sitl_poll_thread,
        shutdown_sitl_transport,
        remove_executor_node,
        shutdown_executor,
        destroy_ros_node,
        shutdown_ros_context,
    ):
        run_shutdown_step(step, bridge)


__all__ = ["initialize_ros2_bridge"]

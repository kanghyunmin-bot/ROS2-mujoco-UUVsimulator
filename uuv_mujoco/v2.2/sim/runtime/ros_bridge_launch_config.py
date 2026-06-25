"""Argument-to-constructor mapping for the ROS2/SITL bridge."""

from __future__ import annotations

from typing import Any, Callable


def ping360_overrides_from_args(args) -> dict[str, Any]:
    return {
        "requested_range_m": args.ping360_range_m,
        "num_steps": args.ping360_num_steps,
        "interface_mode": args.ping360_interface,
        "gain_setting": args.ping360_gain,
    }


def ros_bridge_kwargs_from_args(
    *,
    args,
    model: Any,
    command_callback: Callable[[float, float, float, float], None],
    cmd_limit: float,
) -> dict[str, Any]:
    return {
        "model": model,
        "command_callback": command_callback,
        "cmd_limit": cmd_limit,
        "publish_images": args.ros2_images,
        "image_width": args.ros2_image_width,
        "image_height": args.ros2_image_height,
        "sensor_hz": args.ros2_sensor_hz,
        "image_hz": args.ros2_image_hz,
        "enable_sitl": args.sitl,
        "sitl_ip": args.sitl_ip,
        "sitl_port": args.sitl_port,
        "sitl_send_port": args.sitl_send_port,
        "sitl_mavlink_endpoint": str(args.sitl_mavlink_endpoint),
        "sitl_mavlink_servo_hz": float(args.sitl_mavlink_servo_hz),
        "sitl_mavlink_target_sysid": int(args.sitl_mavlink_target_sysid),
        "sitl_mavlink_target_compid": int(args.sitl_mavlink_target_compid),
        "sitl_mavlink_source_sysid": int(args.sitl_mavlink_source_sysid),
        "sitl_mavlink_source_compid": int(args.sitl_mavlink_source_compid),
        "camera_calib_left": args.ros2_camera_calib_left,
        "camera_calib_right": args.ros2_camera_calib_right,
        "enable_ros": bool(args.ros2),
        "enable_mavros_surface": not args.ros2_real_pkg_compat,
        "enable_ping360": not args.no_ping360,
        "ping360_config_path": args.ping360_config,
        "ping360_overrides": ping360_overrides_from_args(args),
    }


__all__ = ["ping360_overrides_from_args", "ros_bridge_kwargs_from_args"]

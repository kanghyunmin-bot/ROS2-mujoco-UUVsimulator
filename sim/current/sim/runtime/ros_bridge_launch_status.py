"""Launch status output for the ROS2/SITL bridge."""

from __future__ import annotations

from bridge.ros2_topic_registry import build_bridge_topic_summary, build_sitl_transport_summary


def print_ros_bridge_launch_summary(args, *, enable_ros2: bool) -> None:
    if enable_ros2:
        print(
            build_bridge_topic_summary(
                enable_ping360=not args.no_ping360,
                real_pkg_compat=bool(args.ros2_real_pkg_compat),
            ),
            flush=True,
        )
        if args.ros2_images:
            print(
                "[bridge] fixed camera transport enabled: forward stereo + top_up "
                "at 1280x720@10Hz (on demand).",
                flush=True,
            )
        if args.ros2_real_pkg_compat and args.sitl:
            print(
                "[bridge] real package compat: launch external MAVROS with "
                "fcu_url:=udp://0.0.0.0:14551@ "
                "(listen for ArduSub/MAVProxy MAVLink and learn its UDP peer)",
                flush=True,
            )
    elif args.sitl:
        print(
            build_sitl_transport_summary(sitl_mavlink_endpoint=str(args.sitl_mavlink_endpoint)),
            flush=True,
        )


def handle_ros_bridge_launch_error(args, exc: Exception):
    if args.sitl:
        raise SystemExit(f"[runtime] --sitl initialization failed: {exc}") from exc
    if args.ros2:
        print(f"[ros2] bridge init failed: {exc}", flush=True)
    return None


__all__ = ["handle_ros_bridge_launch_error", "print_ros_bridge_launch_summary"]

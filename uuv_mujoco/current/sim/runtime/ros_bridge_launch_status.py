"""Launch status output for the ROS2/SITL bridge."""

from __future__ import annotations

from bridge.ros2_topic_registry import build_bridge_topic_summary, build_sitl_transport_summary
from sim.contracts.ground_truth import (
    UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_WARNING,
)


def print_ros_bridge_launch_summary(args, *, enable_ros2: bool) -> None:
    if enable_ros2:
        print(
            build_bridge_topic_summary(
                enable_ping360=not args.no_ping360,
                real_pkg_compat=bool(args.ros2_real_pkg_compat),
                strict_sitl_sensor_transport=bool(
                    args.ros2 and args.sitl and args.ros2_real_pkg_compat
                ),
                unsafe_legacy_ground_truth_odometry_filtered=bool(
                    getattr(
                        args,
                        "unsafe_legacy_ground_truth_odometry_filtered",
                        False,
                    )
                ),
            ),
            flush=True,
        )
        if bool(
            getattr(
                args,
                "unsafe_legacy_ground_truth_odometry_filtered",
                False,
            )
        ):
            print(
                f"[bridge] WARNING: {UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_WARNING}",
                flush=True,
            )
        if args.ros2_images:
            print(
                "[bridge] ROS2 camera publishing enabled: legacy stereo/color topics and "
                "real-stack /imx219/camera0,1 raw + compressed + camera_info aliases.",
                flush=True,
            )
        if args.ros2_real_pkg_compat and args.sitl:
            print(
                "[bridge] real package compat: launch external MAVROS with "
                "fcu_url:=udp://0.0.0.0:14551@ "
                "(commands/state/local position and fused /mavros/imu/data remain "
                "external; strict /mavros/imu/data_raw and static pressure are "
                "bridge-owned delivery streams)",
                flush=True,
            )
    elif args.sitl:
        print(
            build_sitl_transport_summary(sitl_mavlink_endpoint=str(args.sitl_mavlink_endpoint)),
            flush=True,
        )


def handle_ros_bridge_launch_error(args, exc: Exception):
    if bool(
        getattr(args, "ros2_real_pkg_compat", False)
        and getattr(
            args,
            "unsafe_legacy_ground_truth_odometry_filtered",
            False,
        )
    ):
        raise SystemExit(
            "[runtime] strict real-package compatibility rejected the unsafe "
            f"ground-truth alias: {exc}"
        ) from exc
    if args.sitl:
        raise SystemExit(f"[runtime] --sitl initialization failed: {exc}") from exc
    if args.ros2:
        print(f"[ros2] bridge init failed: {exc}", flush=True)
    return None


__all__ = ["handle_ros_bridge_launch_error", "print_ros_bridge_launch_summary"]

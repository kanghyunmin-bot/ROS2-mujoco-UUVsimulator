"""ROS graph count helpers for GUI backend probing."""

from __future__ import annotations

from typing import Any


def safe_count_publishers(owner: Any, topic: str) -> int:
    try:
        return int(owner.count_publishers(topic))
    except Exception:
        return 0


def safe_count_subscribers(owner: Any, topic: str) -> int:
    try:
        return int(owner.count_subscribers(topic))
    except Exception:
        return 0


def service_ready(client: Any) -> int:
    if client is None:
        return 0
    try:
        return 1 if client.service_is_ready() else 0
    except Exception:
        return 0


def probe_graph_counts(owner: Any) -> dict[str, int]:
    return {
        "vehicle_info_services": service_ready(owner._vehicle_info_client),
        "arm_services": service_ready(owner._arm_client),
        "mode_services": service_ready(owner._mode_client),
        "rc_out_publishers": safe_count_publishers(owner, owner._topic("rc/out")),
        "state_publishers": safe_count_publishers(owner, owner._topic("state")),
        "pose_publishers": safe_count_publishers(owner, owner._topic("local_position/pose")),
        "velocity_body_publishers": safe_count_publishers(owner, owner._topic("local_position/velocity_body")),
        "rc_in_publishers": safe_count_publishers(owner, owner._topic("rc/in")),
        "velocity_local_publishers": safe_count_publishers(owner, owner._topic("local_position/velocity_local")),
        "bridge_imu_publishers": safe_count_publishers(owner, "/imu/data"),
        "bridge_battery_publishers": safe_count_publishers(owner, "/battery"),
        "bridge_rovio_publishers": safe_count_publishers(owner, "/rovio/odometry"),
        "bridge_dvl_odom_publishers": safe_count_publishers(owner, "/dvl/odometry"),
        "bridge_dvl_velocity_publishers": safe_count_publishers(owner, "/dvl/velocity"),
        "bridge_depth_publishers": safe_count_publishers(owner, "/depth"),
        "rc_override_subscribers": safe_count_subscribers(owner, owner._topic("rc/override")),
        "manual_control_subscribers": safe_count_subscribers(owner, owner._topic("manual_control/send")),
    }


__all__ = [
    "probe_graph_counts",
    "safe_count_publishers",
    "safe_count_subscribers",
    "service_ready",
]

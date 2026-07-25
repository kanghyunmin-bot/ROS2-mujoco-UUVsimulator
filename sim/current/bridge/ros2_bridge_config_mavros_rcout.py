"""MAVROS RC output publication policy configuration."""

from __future__ import annotations

import os


def configure_mavros_rcout_policy(bridge: object) -> None:
    bridge._mavros_rc_out_header_stamp_source = os.environ.get(
        "ROS2_UUV_RCOUT_HEADER_STAMP_SOURCE",
        "ros_time",
    ).strip().lower()
    bridge._mavros_rc_out_publish_mode = os.environ.get(
        "ROS2_UUV_RCOUT_PUBLISH_MODE",
        "rate_limited",
    ).strip().lower()
    if bridge._mavros_rc_out_publish_mode not in {"event", "rate_limited"}:
        bridge._mavros_rc_out_publish_mode = "rate_limited"


__all__ = ["configure_mavros_rcout_policy"]

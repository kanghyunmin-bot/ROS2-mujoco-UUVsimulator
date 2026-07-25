"""Ping360 ROS message builders for publish jobs."""

from __future__ import annotations

from typing import Any

from .ros2_publish_ping360_cache import (
    Ping360PublishCache,
    get_ping360_echo_msg,
    get_ping360_image_msg,
    get_ping360_scan_msg,
    get_ping360_status_msg,
)


def build_ping360_publish_builders(self, data, stamp, state: Any) -> dict[str, object]:
    cache = Ping360PublishCache()

    return {
        "ping360_image": lambda: get_ping360_image_msg(cache, self, data, stamp, state),
        "ping360_scan": lambda: get_ping360_scan_msg(cache, self, data, stamp, state),
        "ping360_echo": lambda: get_ping360_echo_msg(cache, self, data, stamp, state),
        "ping360_status": lambda: get_ping360_status_msg(cache, self, stamp, state),
    }

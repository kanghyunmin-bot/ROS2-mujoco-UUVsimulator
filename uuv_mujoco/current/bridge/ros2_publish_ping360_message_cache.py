"""Lazy Ping360 image, scan, and echo message cache."""

from __future__ import annotations

from typing import Any

from .ros2_ping360_messages import (
    build_ping360_echo_msg,
    build_ping360_image_msg,
    build_ping360_scan_msg,
)
from .ros2_publish_ping360_cache_state import Ping360PublishCache
from .ros2_publish_ping360_sample_cache import get_ping360_sample


def get_ping360_image_msg(cache: Ping360PublishCache, owner, data, stamp, state: Any) -> Any | None:
    sample = get_ping360_sample(cache, owner, data, state)
    if sample is None:
        return None
    if cache.image_msg is None:
        cache.image_msg = build_ping360_image_msg(
            owner.Image,
            stamp,
            sample,
            owner._ping360_config,
            owner._ping360_image_renderer,
        )
    return cache.image_msg


def get_ping360_scan_msg(cache: Ping360PublishCache, owner, data, stamp, state: Any) -> Any | None:
    sample = get_ping360_sample(cache, owner, data, state)
    if sample is None:
        return None
    if cache.scan_msg is None:
        cache.scan_msg = build_ping360_scan_msg(
            owner.LaserScan,
            stamp,
            sample,
            owner._ping360_config,
        )
    return cache.scan_msg


def get_ping360_echo_msg(cache: Ping360PublishCache, owner, data, stamp, state: Any) -> Any | None:
    sample = get_ping360_sample(cache, owner, data, state)
    if sample is None:
        return None
    if cache.echo_msg is None:
        cache.echo_msg = build_ping360_echo_msg(
            owner.SonarEcho,
            stamp,
            sample,
            owner._ping360_config,
        )
    return cache.echo_msg


__all__ = ["get_ping360_echo_msg", "get_ping360_image_msg", "get_ping360_scan_msg"]

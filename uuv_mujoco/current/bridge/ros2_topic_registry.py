"""Compatibility facade for ROS2 topic registry and summaries."""

from __future__ import annotations

from .ros2_topic_specs import (
    CORE_BRIDGE_TOPICS,
    MAVROS_SURFACE_TOPIC_LABELS,
    PUBLISHER_SPECS,
    SERVICE_SPECS,
    SUBSCRIBER_SPECS,
    UNSAFE_LEGACY_PUBLISHER_SPECS,
    PublisherSpec,
    ServiceSpec,
    SubscriberSpec,
)
from .ros2_topic_summaries import (
    build_bridge_topic_summary,
    build_ros2_bridge_active_log,
    build_sitl_transport_summary,
    sitl_servo_mode_from_endpoint,
)


__all__ = [
    "PublisherSpec",
    "SubscriberSpec",
    "ServiceSpec",
    "PUBLISHER_SPECS",
    "UNSAFE_LEGACY_PUBLISHER_SPECS",
    "SUBSCRIBER_SPECS",
    "SERVICE_SPECS",
    "CORE_BRIDGE_TOPICS",
    "MAVROS_SURFACE_TOPIC_LABELS",
    "build_bridge_topic_summary",
    "sitl_servo_mode_from_endpoint",
    "build_sitl_transport_summary",
    "build_ros2_bridge_active_log",
]

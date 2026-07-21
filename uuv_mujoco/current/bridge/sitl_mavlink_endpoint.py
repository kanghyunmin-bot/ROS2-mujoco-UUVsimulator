"""MAVLink endpoint helpers for SitlTransport connection setup."""

from __future__ import annotations

import os

DISABLED_ENDPOINT_VALUES = {"none", "off", "disabled", "disable"}


def endpoint_is_disabled(endpoint: str | None) -> bool:
    return bool(endpoint and endpoint.strip().lower() in DISABLED_ENDPOINT_VALUES)


def default_servo_mavlink_endpoint() -> str:
    return f"udpin:0.0.0.0:{int(os.getenv('ROS2_UUV_SITL_MAV_PORT', '14660'))}"


def servo_mavlink_endpoint(self) -> str:
    endpoint = self._sitl_mavlink_endpoint
    if not endpoint:
        endpoint = default_servo_mavlink_endpoint()
        self._sitl_mavlink_endpoint = endpoint
    return endpoint


__all__ = [
    "DISABLED_ENDPOINT_VALUES",
    "default_servo_mavlink_endpoint",
    "endpoint_is_disabled",
    "servo_mavlink_endpoint",
]

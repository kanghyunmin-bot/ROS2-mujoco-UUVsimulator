"""Compatibility exports for MAVROS arm/mode forwarding callbacks."""

from __future__ import annotations

from .ros2_mavros_arm_mode_callbacks import _on_mavros_cmd_arming, _on_mavros_set_mode
from .ros2_mavros_arm_mode_forwarding import _forward_arm_request, _forward_mode_request


__all__ = [
    "_forward_arm_request",
    "_forward_mode_request",
    "_on_mavros_cmd_arming",
    "_on_mavros_set_mode",
]

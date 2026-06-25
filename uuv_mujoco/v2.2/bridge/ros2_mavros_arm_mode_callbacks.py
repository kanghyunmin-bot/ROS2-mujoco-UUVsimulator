"""ROS service callbacks for MAVROS arm/mode compatibility services."""

from __future__ import annotations


def _on_mavros_cmd_arming(self, request, response):
    arm_value = bool(getattr(request, "value", False))
    forward_ok = self._forward_arm_request(arm_value, "/mavros/cmd/arming")
    if hasattr(response, "success"):
        response.success = bool(forward_ok)
    if hasattr(response, "result"):
        response.result = 0 if forward_ok else 1
    return response


def _on_mavros_set_mode(self, request, response):
    mode = str(getattr(request, "custom_mode", ""))
    forward_ok = self._forward_mode_request(mode, "/mavros/set_mode")
    if hasattr(response, "mode_sent"):
        response.mode_sent = bool(forward_ok and mode)
    if hasattr(response, "success"):
        response.success = bool(forward_ok)
    return response


__all__ = ["_on_mavros_cmd_arming", "_on_mavros_set_mode"]

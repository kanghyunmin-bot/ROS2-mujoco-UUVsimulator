"""Internal SITL command override callback for Ros2Bridge."""

from __future__ import annotations

from .ros2_sitl_command_override_arm_mode import _handle_arm_override, _handle_mode_override
from .ros2_sitl_command_override_replay import _handle_replay_rcout
from .ros2_sitl_command_override_sensor import _mark_sensor_replay_input
from .ros2_sitl_command_override_topic import COMMAND_OVERRIDE_TOPIC


def _float_payload_value(payload: dict[str, object], *keys: str, default: float = 0.0) -> float:
    for key in keys:
        if key not in payload:
            continue
        try:
            return max(-1.0, min(1.0, float(payload[key])))
        except Exception:
            return float(default)
    return float(default)


def _handle_direct_cmd(self, payload: dict[str, object]) -> None:
    direct = payload.get("direct_cmd", payload.get("cmd"))
    if not isinstance(direct, dict):
        return
    forward = _float_payload_value(direct, "forward", "fwd", "x")
    sway = _float_payload_value(direct, "sway", "left", "y")
    heave = _float_payload_value(direct, "heave", "up", "z")
    yaw = _float_payload_value(direct, "yaw", "r")
    if self._sitl_transport is not None and not getattr(self._sitl_transport, "vehicle_armed", False):
        return
    self._handle_normalized_cmd(forward, sway, yaw, heave)


def _on_sitl_command_override(self, msg) -> None:
    payload = self._parse_command_override_payload(getattr(msg, "data", ""))
    if not payload:
        return

    _mark_sensor_replay_input(self, payload)
    _handle_replay_rcout(self, payload)
    _handle_direct_cmd(self, payload)
    _handle_arm_override(self, payload)
    _handle_mode_override(self, payload)


__all__ = ["COMMAND_OVERRIDE_TOPIC", "_handle_direct_cmd", "_on_sitl_command_override"]

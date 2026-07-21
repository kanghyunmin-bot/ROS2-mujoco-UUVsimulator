"""MAVROS state message builder for Ros2Bridge."""

from __future__ import annotations

from .ros2_runtime_mavros_state_header import fill_state_header
from .ros2_runtime_mavros_state_values import mavros_state_values


def build_mavros_state(self, stamp):
    msg = self.MavrosState()
    fill_state_header(msg, stamp)
    armed, mode, connected, manual_input = mavros_state_values(self)
    if hasattr(msg, "connected"):
        msg.connected = bool(connected)
    if hasattr(msg, "armed"):
        msg.armed = armed
    if hasattr(msg, "guided"):
        msg.guided = mode == "GUIDED"
    if hasattr(msg, "manual_input"):
        msg.manual_input = bool(manual_input)
    if hasattr(msg, "mode"):
        msg.mode = str(mode)
    if hasattr(msg, "system_status"):
        msg.system_status = 0
    return msg


__all__ = ["build_mavros_state"]

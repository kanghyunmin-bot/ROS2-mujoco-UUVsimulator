"""Vehicle HEARTBEAT and COMMAND_ACK compatibility exports."""

from __future__ import annotations

from .sitl_vehicle_command_ack import _handle_command_ack
from .sitl_vehicle_heartbeat_filter import _heartbeat_is_vehicle
from .sitl_vehicle_state import _update_vehicle_heartbeat


__all__ = [
    "_handle_command_ack",
    "_heartbeat_is_vehicle",
    "_update_vehicle_heartbeat",
]

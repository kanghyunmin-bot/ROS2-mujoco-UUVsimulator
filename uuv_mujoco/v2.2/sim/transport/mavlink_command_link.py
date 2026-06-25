"""Low-level MAVLink command-link helpers for ArduSub SITL."""

from __future__ import annotations

from dataclasses import dataclass

from .mavlink_command_endpoint import DISABLED_COMMAND_ENDPOINTS, command_endpoint_disabled
from .mavlink_command_link_connection import MavlinkCommandLinkConnectionMixin
from .mavlink_command_link_sends import MavlinkCommandLinkSendMixin
from .mavlink_command_link_state import MavlinkCommandLinkStateMixin


@dataclass
class MavlinkCommandLink(
    MavlinkCommandLinkStateMixin,
    MavlinkCommandLinkConnectionMixin,
    MavlinkCommandLinkSendMixin,
):
    """Own low-level command-link connection and send primitives."""

    endpoint: str = ""
    source_system: int = 255
    source_component: int = 190
    mav: object | None = None
    last_connect_attempt_wall: float = -1.0
    last_heartbeat_send_wall: float = -1.0


__all__ = [
    "DISABLED_COMMAND_ENDPOINTS",
    "MavlinkCommandLink",
    "command_endpoint_disabled",
]

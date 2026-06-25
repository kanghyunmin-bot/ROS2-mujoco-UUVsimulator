"""Vehicle state updates derived from ArduSub HEARTBEAT messages."""

from __future__ import annotations

import time


from .sitl_vehicle_heartbeat_filter import _heartbeat_is_vehicle
from .sitl_vehicle_state_extract import (
    heartbeat_armed,
    heartbeat_mode,
    record_vehicle_state_from_heartbeat,
)
from .sitl_vehicle_state_log import log_vehicle_state_change


def _update_vehicle_heartbeat(self, msg, *, command_link: bool = False) -> None:
    if not _heartbeat_is_vehicle(self, msg):
        return
    if command_link:
        self._sitl_cmd_mav_hb = msg
        self._sitl_cmd_mav_last_hb_wall = time.monotonic()
    else:
        self._sitl_mav_hb = msg
        self._sitl_mav_last_hb_wall = time.monotonic()
    _record_vehicle_state_from_heartbeat(self, msg, command_link=command_link)


def _record_vehicle_state_from_heartbeat(self, msg, *, command_link: bool) -> None:
    record_vehicle_state_from_heartbeat(self, msg, command_link=command_link)


def _heartbeat_armed(self, msg) -> bool:
    return heartbeat_armed(self, msg)


def _heartbeat_mode(self, msg) -> str:
    return heartbeat_mode(self, msg)


def _log_vehicle_state_change(self, *, armed: bool, mode: str, command_link: bool) -> None:
    log_vehicle_state_change(self, armed=armed, mode=mode, command_link=command_link)


__all__ = ["_update_vehicle_heartbeat"]

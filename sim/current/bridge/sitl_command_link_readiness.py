"""Readiness checks for SITL MAVLink command links."""

from __future__ import annotations

import time

from .sitl_command_link_activity import (
    any_heartbeat_seen,
    any_mavlink_object_has_clients,
    recent_wall_activity,
)
from .sitl_command_link_select import _mav_for_commands
from .sitl_command_target_resolution import _resolve_mav_target


@property
def mavlink_connected(self) -> bool:
    now = time.monotonic()
    if any_heartbeat_seen(self._sitl_cmd_mav_hb, self._sitl_mav_hb):
        return True
    if recent_wall_activity(self._sitl_mav_last_msg_wall, now):
        return True
    return any_mavlink_object_has_clients((self._sitl_cmd_mav, self._sitl_mav))


def _selected_link_recent_heartbeat(self, mav: object, now: float) -> bool:
    if mav is self._sitl_cmd_mav:
        return recent_wall_activity(self._sitl_cmd_mav_last_hb_wall, now)
    return recent_wall_activity(self._sitl_mav_last_hb_wall, now)


def _selected_link_has_recent_activity(self, mav: object, now: float) -> bool:
    if _selected_link_recent_heartbeat(self, mav, now):
        return True
    if mav is self._sitl_cmd_mav:
        if recent_wall_activity(getattr(self, "_sitl_cmd_servo_last_msg_wall", -1.0), now):
            return True
    elif recent_wall_activity(self._sitl_mav_last_msg_wall, now):
        return True
    return any_mavlink_object_has_clients((mav,))


@property
def rc_override_ready(self) -> bool:
    """True only when the MAVLink command path can accept pilot RC input."""
    mav = _mav_for_commands(self)
    if mav is None:
        return False
    if _resolve_mav_target(self, mav) is None:
        return False
    now = time.monotonic()
    return _selected_link_has_recent_activity(self, mav, now)


__all__ = ["mavlink_connected", "rc_override_ready"]

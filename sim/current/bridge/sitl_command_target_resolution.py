"""MAVLink target resolution for SITL command messages."""

from __future__ import annotations

from .sitl_command_target_heartbeat import assign_mav_target, fill_target_from_heartbeat, heartbeat_for_command_mav
from .sitl_command_link_select import _mav_for_commands


def _resolve_mav_target(self, mav=None) -> tuple[int, int] | None:
    mav = mav if mav is not None else _mav_for_commands(self)
    if mav is None:
        return None
    target_sys = int(self._sitl_mavlink_target_sysid)
    target_comp = int(self._sitl_mavlink_target_compid)
    if target_sys <= 0:
        target = fill_target_from_heartbeat(target_sys, target_comp, heartbeat_for_command_mav(self, mav))
        if target is None:
            return None
        target_sys, target_comp = target
    assign_mav_target(mav, target_sys, target_comp)
    return target_sys, target_comp


__all__ = ["_resolve_mav_target"]

"""Target matching helpers for SITL vehicle HEARTBEAT messages."""

from __future__ import annotations


def heartbeat_source_matches_target(self, msg) -> bool:
    src_sys = int(msg.get_srcSystem())
    src_comp = int(msg.get_srcComponent())
    target_sys = int(self._sitl_mavlink_target_sysid)
    target_comp = int(self._sitl_mavlink_target_compid)
    if target_sys > 0 and src_sys != target_sys:
        return False
    if target_comp > 0 and src_comp != target_comp:
        return False
    return True


def heartbeat_autopilot_matches(self, msg) -> bool:
    autopilot_mega = int(self._sitl_mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
    return int(getattr(msg, "autopilot", -1)) == autopilot_mega


__all__ = ["heartbeat_autopilot_matches", "heartbeat_source_matches_target"]

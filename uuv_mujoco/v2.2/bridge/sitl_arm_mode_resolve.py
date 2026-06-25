"""ArduSub mode text resolution for SitlTransport."""

from __future__ import annotations

from sim.transport import resolve_ardusub_mode_id


def _mode_id_for_text(self, mode: str) -> tuple[str, int] | None:
    mode_text = str(mode).strip()
    if not mode_text:
        return None
    try:
        mavlink_defs = self._sitl_mavutil.mavlink
        mode_map = self._sitl_mavutil.mode_mapping_byname(mavlink_defs.MAV_TYPE_SUBMARINE) or {}
    except Exception:
        mode_map = {}
    return resolve_ardusub_mode_id(mode_text, mode_map)


__all__ = ["_mode_id_for_text"]

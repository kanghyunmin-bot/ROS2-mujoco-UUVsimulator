"""MAVLink telemetry observer wrappers for SitlTransport."""

from __future__ import annotations


def _mavlink_source_matches_target(self, msg) -> bool:
    return self._mavlink_telemetry_observer.source_matches_target(msg)


def _store_ap_mavlink_telemetry(self, msg, now_wall: float) -> None:
    self._mavlink_telemetry_observer.observe(msg, now_wall)


__all__ = ["_mavlink_source_matches_target", "_store_ap_mavlink_telemetry"]

"""Passive MAVLink telemetry observer facade for ArduSub SITL."""

from __future__ import annotations

from dataclasses import dataclass, field

from .mavlink_telemetry_dispatch import observe_mavlink_telemetry
from .mavlink_telemetry_status import TELEMETRY_STATUS_PREFIXES, build_status_snapshot
from .mavlink_telemetry_target import source_matches_target_ids


@dataclass
class MavlinkTelemetryObserver:
    """Store passive MAVLink telemetry without owning a command link."""

    target_system: int = 0
    target_component: int = 0
    endpoint: str = ""
    requested_hz: float = 0.0
    status_data: dict[str, object] = field(default_factory=dict)

    def set_endpoint(self, endpoint: str) -> None:
        self.endpoint = str(endpoint or "")

    def source_matches_target(self, msg: object) -> bool:
        return source_matches_target_ids(
            msg,
            target_system=self.target_system,
            target_component=self.target_component,
        )

    def observe(self, msg: object, now_wall: float) -> None:
        observe_mavlink_telemetry(
            msg,
            now_wall,
            status_data=self.status_data,
            source_matches_target=self.source_matches_target,
        )

    def status_snapshot(self, now_wall: float) -> dict[str, object]:
        return build_status_snapshot(
            self.status_data,
            now_wall,
            endpoint=self.endpoint,
            requested_hz=self.requested_hz,
        )


__all__ = [
    "MavlinkTelemetryObserver",
    "TELEMETRY_STATUS_PREFIXES",
]

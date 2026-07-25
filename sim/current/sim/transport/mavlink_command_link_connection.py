"""Connection helpers for the low-level MAVLink command link."""

from __future__ import annotations


class MavlinkCommandLinkConnectionMixin:
    """MAVLink connection and reconnect-cadence helpers."""

    def connect(self, mavutil: object) -> object | None:
        if self.disabled:
            return None
        connection = mavutil.mavlink_connection(
            self.endpoint,
            source_system=int(self.source_system),
            source_component=int(self.source_component),
            force_mavlink1=False,
            autoreconnect=True,
        )
        self.mav = connection
        return connection

    def should_attempt_reconnect(self, now_wall: float, *, min_interval_s: float = 2.0) -> bool:
        if self.mav is not None or self.disabled:
            return False
        if float(now_wall) - float(self.last_connect_attempt_wall) < float(min_interval_s):
            return False
        self.last_connect_attempt_wall = float(now_wall)
        return True


__all__ = ["MavlinkCommandLinkConnectionMixin"]

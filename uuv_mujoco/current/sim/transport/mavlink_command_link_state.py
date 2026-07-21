"""State helpers for the low-level MAVLink command link."""

from __future__ import annotations

from .mavlink_command_endpoint import command_endpoint_disabled


class MavlinkCommandLinkStateMixin:
    """Endpoint and connection state helpers."""

    @property
    def disabled(self) -> bool:
        return command_endpoint_disabled(self.endpoint)

    def set_endpoint(self, endpoint: str) -> None:
        self.endpoint = str(endpoint or "").strip()

    def set_connection(self, mav: object | None) -> None:
        self.mav = mav


__all__ = ["MavlinkCommandLinkStateMixin"]

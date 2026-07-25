"""Ping360 status callbacks for UuvGuiNode."""

from __future__ import annotations

from .node_ping360_status_payload import parse_ping360_payload, ping360_bool_fields
from .node_ping360_status_settings import extract_ping360_status_settings
from .node_ping360_status_summary import format_ping360_summary


def _on_ping360_status(self, msg: String) -> None:
    self._touch("ping360")
    payload, settings = parse_ping360_payload(msg)
    active_bool, enabled_bool = ping360_bool_fields(payload)
    status = extract_ping360_status_settings(payload, settings)
    summary = format_ping360_summary(status, active_bool=active_bool, enabled_bool=enabled_bool)
    with self._lock:
        self._snapshot.ping360_summary = summary
        self._snapshot.ping360_enabled = enabled_bool
        self._snapshot.ping360_active = active_bool


__all__ = ["_on_ping360_status"]

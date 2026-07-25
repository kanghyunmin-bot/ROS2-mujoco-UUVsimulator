"""Payload parsing helpers for Ping360 GUI status callbacks."""

from __future__ import annotations

import json


def parse_ping360_payload(msg) -> tuple[dict, dict]:
    try:
        payload = json.loads(str(msg.data))
    except json.JSONDecodeError:
        return {}, {}
    if not isinstance(payload, dict):
        return {}, {}
    settings = payload.get("settings", {})
    return payload, settings if isinstance(settings, dict) else {}


def ping360_bool_fields(payload: dict) -> tuple[bool | None, bool | None]:
    active = payload.get("active")
    enabled = payload.get("enabled")
    active_bool = bool(active) if isinstance(active, bool) else None
    enabled_bool = bool(enabled) if isinstance(enabled, bool) else None
    return active_bool, enabled_bool


__all__ = ["parse_ping360_payload", "ping360_bool_fields"]

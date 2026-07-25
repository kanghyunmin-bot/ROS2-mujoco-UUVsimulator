"""MAVLink command endpoint state helpers."""

from __future__ import annotations


DISABLED_COMMAND_ENDPOINTS = {"", "servo", "same", "none", "disabled", "off"}


def command_endpoint_disabled(endpoint: str) -> bool:
    return str(endpoint or "").strip().lower() in DISABLED_COMMAND_ENDPOINTS


__all__ = ["DISABLED_COMMAND_ENDPOINTS", "command_endpoint_disabled"]

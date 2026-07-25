"""MAVLink dependency loading for SitlTransport."""

from __future__ import annotations


def require_pymavlink_mavutil():
    try:
        from pymavlink import mavutil
    except Exception as exc:
        raise RuntimeError("pymavlink is required for SITL MAVLink servo input") from exc
    return mavutil


__all__ = ["require_pymavlink_mavutil"]

"""Vehicle-info refresh helpers for the GUI update loop."""

from __future__ import annotations


def request_vehicle_info_if_due(owner, *, now: float, interval_s: float = 2.0) -> None:
    if now - owner._last_vehicle_info_wall <= interval_s:
        return
    owner.node.request_vehicle_info()
    owner._last_vehicle_info_wall = now


__all__ = ["request_vehicle_info_if_due"]

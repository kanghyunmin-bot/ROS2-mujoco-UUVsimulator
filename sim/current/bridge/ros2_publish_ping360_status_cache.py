"""Lazy Ping360 status message cache."""

from __future__ import annotations

from typing import Any

from .ros2_ping360_messages import build_ping360_status_payload, build_ping360_status_msg
from .ros2_publish_ping360_cache_state import Ping360PublishCache


def get_ping360_status_msg(cache: Ping360PublishCache, owner, stamp, state: Any) -> Any | None:
    if state.sim_t + 1.0e-9 < owner._ping360_status_next_t:
        return None
    owner._ping360_status_next_t = state.sim_t + owner._ping360_status_period_s
    if cache.status_msg is None:
        payload = build_ping360_status_payload(
            sim_t=state.sim_t,
            config=owner._ping360_config,
            ping360=owner._ping360,
            sample=cache.sample,
            site_present=bool(owner._ping360_site_id >= 0),
        )
        cache.status_msg = build_ping360_status_msg(owner.String, stamp, payload)
    return cache.status_msg


__all__ = ["get_ping360_status_msg"]

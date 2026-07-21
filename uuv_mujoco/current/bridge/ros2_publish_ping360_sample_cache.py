"""Lazy Ping360 sample cache."""

from __future__ import annotations

from typing import Any

from .ros2_publish_ping360_cache_state import Ping360PublishCache


def get_ping360_sample(cache: Ping360PublishCache, owner, data, state: Any) -> Any | None:
    if owner._ping360 is None or not owner._ping360.active:
        return None
    if cache.sample is None:
        cache.sample = owner._ping360.update(data, state.sim_t)
    return cache.sample


__all__ = ["get_ping360_sample"]

"""Compatibility facade for ExternalNav/VISION_POSITION_DELTA runtime helpers."""

from __future__ import annotations

from bridge.sitl_external_nav_bootstrap import _send_external_nav_bootstrap
from bridge.sitl_external_nav_cache_contract import (
    _cache_external_nav_state,
    _enforce_extnav_contract,
    live_wall_external_nav,
    send_cached_external_nav_due,
)
from bridge.sitl_external_nav_synthetic_vpd import _send_external_nav
from bridge.sitl_native_vpd_runtime import _send_native_vision_delta_due


__all__ = [
    "_cache_external_nav_state",
    "_enforce_extnav_contract",
    "_send_external_nav",
    "_send_external_nav_bootstrap",
    "_send_native_vision_delta_due",
    "live_wall_external_nav",
    "send_cached_external_nav_due",
]

"""ExternalNav/VPD method bindings for SitlTransport."""

from __future__ import annotations

from bridge import sitl_external_nav_runtime


class SitlTransportExternalNavBindings:
    _send_external_nav_bootstrap = sitl_external_nav_runtime._send_external_nav_bootstrap
    _send_native_vision_delta_due = sitl_external_nav_runtime._send_native_vision_delta_due
    _send_external_nav = sitl_external_nav_runtime._send_external_nav
    _cache_external_nav_state = sitl_external_nav_runtime._cache_external_nav_state
    send_cached_external_nav_due = sitl_external_nav_runtime.send_cached_external_nav_due
    live_wall_external_nav = sitl_external_nav_runtime.live_wall_external_nav
    _enforce_extnav_contract = sitl_external_nav_runtime._enforce_extnav_contract


__all__ = ["SitlTransportExternalNavBindings"]

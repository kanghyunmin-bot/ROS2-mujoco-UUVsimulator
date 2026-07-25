"""Composed method-binding surface for SitlTransport."""

from __future__ import annotations

from .sitl_transport_command_bindings import SitlTransportCommandBindings
from .sitl_transport_extnav_bindings import SitlTransportExternalNavBindings
from .sitl_transport_handler_bindings import SitlTransportHandlerBindings
from .sitl_transport_json_bindings import SitlTransportJsonBindings
from .sitl_transport_mavlink_bindings import SitlTransportMavlinkBindings
from .sitl_transport_state_bindings import SitlTransportStateBindings


class SitlTransportBindings(
    SitlTransportStateBindings,
    SitlTransportHandlerBindings,
    SitlTransportCommandBindings,
    SitlTransportMavlinkBindings,
    SitlTransportExternalNavBindings,
    SitlTransportJsonBindings,
):
    """Aggregate existing runtime helper bindings without changing names."""


__all__ = ["SitlTransportBindings"]

"""Initialization facade for the GUI ROS node."""

from __future__ import annotations

from .config import BACKEND_AUTO, DEFAULT_AUTO_BACKEND
from .helpers import normalize_backend_name
from .node_init_publishers import initialize_command_publishers_and_clients
from .node_init_state import (
    initialize_command_contract_state,
    initialize_node_state,
    initialize_real_start_state,
)
from .node_init_subscriptions import initialize_subscriptions
from .runtime import HAVE_MAVROS_MSGS


def initialize_uuv_gui_node(self, namespace: str, backend: str) -> None:
    ns = namespace.rstrip("/")
    self._base_ns = ns if ns else ""
    self._backend_preference = normalize_backend_name(backend)
    self._backend_detected = (
        self._backend_preference
        if self._backend_preference != BACKEND_AUTO
        else DEFAULT_AUTO_BACKEND
    )

    initialize_node_state(self)
    initialize_command_contract_state(self)
    initialize_real_start_state(self)
    initialize_command_publishers_and_clients(self)
    initialize_subscriptions(self)
    announce_node_startup(self)


def announce_node_startup(self) -> None:
    self._push_event(
        f"GUI attached to {self._base_ns or '/mavros'} "
        f"(pilot joystick via {self._topic('manual_control/send')}; "
        f"rosbag replay via {self._topic('rc/override')})"
    )
    if not HAVE_MAVROS_MSGS:
        self._push_event("mavros_msgs not available in this Python env: MAVROS arm/mode/RC features disabled")
    self._probe_backend(force=True)


__all__ = [
    "initialize_uuv_gui_node",
    "announce_node_startup",
    "initialize_node_state",
    "initialize_command_contract_state",
    "initialize_real_start_state",
    "initialize_command_publishers_and_clients",
    "initialize_subscriptions",
]

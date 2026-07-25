"""SITL MAVLink servo/telemetry connection setup."""

from __future__ import annotations

from .sitl_mavlink_endpoint import endpoint_is_disabled, servo_mavlink_endpoint
from .sitl_mavlink_imports import require_pymavlink_mavutil


def _connect_sitl_mavlink(self) -> None:
    """Initialize MAVLink input channel for SITL servo outputs."""
    if endpoint_is_disabled(self._sitl_mavlink_endpoint):
        self._sitl_mav = None
        self._sitl_mavutil = None
        print(
            "[sitl_transport] SITL MAVLink servo input disabled; using JSON UDP servo packets only.",
            flush=True,
        )
        return

    mavutil = require_pymavlink_mavutil()
    endpoint = servo_mavlink_endpoint(self)
    configure_servo_mavlink_observers(self, endpoint)
    self._sitl_mav = mavutil.mavlink_connection(
        endpoint,
        source_system=self._sitl_mavlink_source_system,
        source_component=self._sitl_mavlink_source_component,
        force_mavlink1=False,
        autoreconnect=True,
    )
    self._sitl_servo_command_link.set_connection(self._sitl_mav)
    self._sitl_mavutil = mavutil
    print(f"[sitl_transport] SITL MAVLink servo input enabled: endpoint={endpoint}", flush=True)
    log_json_fallback_source_policy(self)
    self._connect_sitl_command_mavlink(mavutil)


def configure_servo_mavlink_observers(self, endpoint: str) -> None:
    self._mavlink_telemetry_observer.set_endpoint(endpoint)
    self._sitl_servo_command_link.set_endpoint(endpoint)


def log_json_fallback_source_policy(self) -> None:
    if self._sitl_json_servo_fallback:
        print(
            "[sitl_transport] SITL JSON servo packets are the active thruster source; "
            "MAVLink SERVO_OUTPUT_RAW is used passively for vehicle state/telemetry only.",
            flush=True,
        )


__all__ = [
    "_connect_sitl_mavlink",
    "configure_servo_mavlink_observers",
    "log_json_fallback_source_policy",
]

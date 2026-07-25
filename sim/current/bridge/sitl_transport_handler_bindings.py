"""Servo-handler and lifecycle method bindings for SitlTransport."""

from __future__ import annotations

from bridge import sitl_transport_handlers, sitl_transport_lifecycle


class SitlTransportHandlerBindings:
    set_servo_handler = sitl_transport_handlers.set_servo_handler
    set_servo_telemetry_handler = sitl_transport_handlers.set_servo_telemetry_handler
    inject_servo_pwm_values = sitl_transport_handlers.inject_servo_pwm_values

    _connect_sitl = sitl_transport_lifecycle._connect_sitl
    shutdown = sitl_transport_lifecycle.shutdown


__all__ = ["SitlTransportHandlerBindings"]

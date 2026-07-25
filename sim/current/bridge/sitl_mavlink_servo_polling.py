"""MAVLink SERVO_OUTPUT_RAW polling loop for SitlTransport."""

from __future__ import annotations

import time

from .qgc_mavlink_relay import drain_qgc_mavlink_relay
from .sitl_mavlink_servo_drain import drain_servo_mavlink
from .sitl_mavlink_servo_handlers import _warn_waiting_for_servo_output


def _poll_servo_mavlink(self) -> None:
    if self._sitl_mav is None:
        return
    now_wall = time.monotonic()
    self._service_pending_arm_command(now_wall)
    self._service_pending_mode_command(now_wall)
    drain_qgc_mavlink_relay(self)
    got_any, latest_pwm_values = drain_servo_mavlink(self, now_wall)

    if latest_pwm_values is not None:
        self._handle_pwm_values(latest_pwm_values, now_wall, source="mavlink")

    # On a shared serial2 path, SET_MESSAGE_INTERVAL ACKs can crowd out
    # arm/mode command ACKs. Pause stream requests while operator-critical
    # arm/mode commands are pending; telemetry resumes immediately afterwards.
    if not self._arm_mode_command_pending():
        self._request_sitl_mavlink_servo_stream()
        self._request_sitl_mavlink_ap_telemetry_stream()
    _warn_waiting_for_servo_output(self, got_any, now_wall)

__all__ = ["_poll_servo_mavlink"]

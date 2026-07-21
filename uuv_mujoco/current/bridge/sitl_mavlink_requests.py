"""MAVLink stream request helpers for SitlTransport."""

from __future__ import annotations

import time

from .sitl_mavlink_request_ap import request_command_link_ap_telemetry, request_servo_link_ap_telemetry
from .sitl_mavlink_request_servo import request_command_link_servo_stream, request_servo_link_servo_stream


def _request_sitl_mavlink_servo_stream(self) -> None:
    request_servo_link_servo_stream(self, now_wall=time.monotonic())


def _request_command_servo_telemetry_stream(self, now_wall: float) -> None:
    request_command_link_servo_stream(self, now_wall=now_wall)


def _request_sitl_mavlink_ap_telemetry_stream(self) -> None:
    request_servo_link_ap_telemetry(self, now_wall=time.monotonic())


def _request_command_ap_telemetry_stream(self, now_wall: float) -> None:
    request_command_link_ap_telemetry(self, now_wall=now_wall)

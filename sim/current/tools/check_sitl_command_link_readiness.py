#!/usr/bin/env python3
"""Smoke checks for SITL command-link readiness predicates."""

from __future__ import annotations

import pathlib
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.sitl_command_link_activity import recent_wall_activity  # noqa: E402
from bridge.sitl_command_link_readiness import mavlink_connected, rc_override_ready  # noqa: E402


class FakeMav:
    def __init__(self, clients=None) -> None:
        self.clients = clients
        self.target_system = 1
        self.target_component = 1


class FakeTransport:
    def __init__(self) -> None:
        self._sitl_cmd_mav = None
        self._sitl_mav = None
        self._sitl_cmd_mav_hb = None
        self._sitl_mav_hb = None
        self._sitl_mav_last_msg_wall = 0.0
        self._sitl_cmd_mav_last_hb_wall = 0.0
        self._sitl_mav_last_hb_wall = 0.0
        self._sitl_cmd_servo_last_msg_wall = 0.0
        self._sitl_command_prefer_dedicated = True
        self._sitl_target_system = 0
        self._sitl_target_component = 0
        self._sitl_cmd_target_system = 0
        self._sitl_cmd_target_component = 0
        self._sitl_mavlink_target_sysid = 1
        self._sitl_mavlink_target_compid = 1


FakeTransport.mavlink_connected = mavlink_connected
FakeTransport.rc_override_ready = rc_override_ready


def _assert_true(value: bool, label: str) -> None:
    if not value:
        raise AssertionError(label)


def _assert_false(value: bool, label: str) -> None:
    if value:
        raise AssertionError(label)


def main() -> int:
    now = 10.0
    _assert_true(recent_wall_activity(now - 1.0, now), "recent wall activity")
    _assert_false(recent_wall_activity(now - 4.0, now), "stale wall activity")

    transport = FakeTransport()
    _assert_false(transport.mavlink_connected, "no link is not connected")
    transport._sitl_mav_last_msg_wall = time.monotonic()
    _assert_true(transport.mavlink_connected, "recent passive MAVLink message connects")

    transport = FakeTransport()
    transport._sitl_cmd_mav = FakeMav(clients={("127.0.0.1", 14550)})
    _assert_true(transport.mavlink_connected, "client list connects")
    _assert_true(transport.rc_override_ready, "client list and explicit target are RC-ready")

    transport = FakeTransport()
    transport._sitl_cmd_mav = FakeMav(clients=None)
    _assert_false(transport.rc_override_ready, "no command-link activity is not RC-ready")
    transport._sitl_cmd_servo_last_msg_wall = time.monotonic()
    _assert_true(transport.rc_override_ready, "dedicated recent telemetry and target are RC-ready")

    transport = FakeTransport()
    transport._sitl_cmd_mav = FakeMav(clients=None)
    transport._sitl_cmd_mav_last_hb_wall = time.monotonic()
    _assert_true(transport.rc_override_ready, "dedicated heartbeat and target are RC-ready")

    print("sitl_command_link_readiness=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

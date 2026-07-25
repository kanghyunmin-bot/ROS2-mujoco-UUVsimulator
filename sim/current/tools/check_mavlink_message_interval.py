#!/usr/bin/env python3
"""Smoke-check throttled MAVLink SET_MESSAGE_INTERVAL requests."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.transport.mavlink_message_interval import MavlinkMessageIntervalRequester  # noqa: E402


class FakeMavlinkDefs:
    MAV_CMD_SET_MESSAGE_INTERVAL = 511
    MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 36
    MAVLINK_MSG_ID_ATTITUDE = 30


class FakeMavutil:
    mavlink = FakeMavlinkDefs


class FakeMav:
    def __init__(self) -> None:
        self.mav = self
        self.commands: list[tuple[object, ...]] = []

    def command_long_send(self, *args) -> None:
        self.commands.append(args)


def test_servo_output_raw_throttle() -> None:
    requester = MavlinkMessageIntervalRequester()
    mav = FakeMav()
    assert requester.request_servo_output_raw(
        key="servo",
        mav=mav,
        mavutil=FakeMavutil(),
        target_sys=1,
        target_comp=1,
        now_wall=10.0,
        requested_hz=20.0,
        period_s=5.0,
    )
    assert not requester.request_servo_output_raw(
        key="servo",
        mav=mav,
        mavutil=FakeMavutil(),
        target_sys=1,
        target_comp=1,
        now_wall=12.0,
        requested_hz=20.0,
        period_s=5.0,
    )
    assert requester.last_wall("servo") == 10.0
    assert len(mav.commands) == 1
    assert mav.commands[0][4] == 36.0
    assert mav.commands[0][5] == 50000.0


def test_named_messages_mark_only_on_send() -> None:
    requester = MavlinkMessageIntervalRequester()
    mav = FakeMav()
    assert not requester.request_named_messages(
        key="missing",
        mav=mav,
        mavutil=FakeMavutil(),
        target_sys=1,
        target_comp=1,
        now_wall=10.0,
        requested_hz=10.0,
        period_s=5.0,
        message_constant_names=("MISSING_MESSAGE",),
    )
    assert requester.last_wall("missing") == -1.0
    assert requester.request_named_messages(
        key="att",
        mav=mav,
        mavutil=FakeMavutil(),
        target_sys=1,
        target_comp=1,
        now_wall=11.0,
        requested_hz=10.0,
        period_s=5.0,
        message_constant_names=("MAVLINK_MSG_ID_ATTITUDE",),
    )
    assert requester.last_wall("att") == 11.0
    assert mav.commands[-1][4] == 30.0


def main() -> int:
    test_servo_output_raw_throttle()
    test_named_messages_mark_only_on_send()
    print("mavlink_message_interval=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

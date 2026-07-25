#!/usr/bin/env python3
"""Smoke checks for ROS2 command override payload parsing."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_command_payload import (  # noqa: E402
    _parse_command_bool,
    _parse_command_override_payload,
    bind_command_payload_parsers,
)


def _assert_equal(actual: object, expected: object, label: str) -> None:
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


def main() -> int:
    for value in (True, 1, 1.0, "true", "yes", "on", "arm", "armed"):
        _assert_equal(_parse_command_bool(value), True, f"true value {value!r}")
    for value in (False, 0, 0.0, "false", "no", "off", "disarm", "standby"):
        _assert_equal(_parse_command_bool(value), False, f"false value {value!r}")
    _assert_equal(_parse_command_bool("", default=True), True, "empty default true")
    _assert_equal(_parse_command_bool("", default=False), False, "empty default false")

    _assert_equal(
        _parse_command_override_payload('{"arm": true, "mode": "ALT_HOLD"}'),
        {"arm": True, "mode": "ALT_HOLD"},
        "json dict",
    )
    _assert_equal(_parse_command_override_payload("[1, 2, 3]"), {}, "non-dict json")
    _assert_equal(_parse_command_override_payload(""), {}, "empty payload")
    _assert_equal(
        _parse_command_override_payload("ARM=true mode:MANUAL replay_rcout=1,2,3"),
        {"arm": "true", "mode": "MANUAL", "replay_rcout": "1"},
        "token payload",
    )

    class BoundBridge:
        pass

    bind_command_payload_parsers(BoundBridge)
    bridge = BoundBridge()
    _assert_equal(bridge._parse_command_bool("arm"), True, "bound bool parser")
    _assert_equal(
        bridge._parse_command_override_payload("arm=true mode:MANUAL"),
        {"arm": "true", "mode": "MANUAL"},
        "bound payload parser",
    )

    print("ros2_command_payload=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

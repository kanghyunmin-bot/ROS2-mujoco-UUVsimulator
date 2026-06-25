#!/usr/bin/env python3
"""Regression checks for command-path latency defaults."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_topic_specs import SUBSCRIBER_SPECS  # noqa: E402
from bridge.sitl_arm_mode_service_state import (  # noqa: E402
    ARM_PENDING_RESEND_PERIOD_S,
    MODE_PENDING_RESEND_PERIOD_S,
)


COMMAND_TOPICS = {
    "/cmd_vel",
    "/uuv_mujoco/sitl/command_override",
    "/mavros/rc/override",
    "/mavros/manual_control/send",
    "/mavros/setpoint_raw/local",
}


def _assert(condition: bool, label: str) -> None:
    if not condition:
        raise AssertionError(label)


def check_resend_periods() -> None:
    _assert(ARM_PENDING_RESEND_PERIOD_S <= 0.05, "arm resend period must be <= 50 ms")
    _assert(MODE_PENDING_RESEND_PERIOD_S <= 0.05, "mode resend period must be <= 50 ms")


def check_command_subscriber_depths() -> None:
    by_topic = {spec.topic: spec.qsize for spec in SUBSCRIBER_SPECS}
    for topic in COMMAND_TOPICS & by_topic.keys():
        _assert(by_topic[topic] == 1, f"{topic} subscriber depth must be 1")


def main() -> int:
    check_resend_periods()
    check_command_subscriber_depths()
    print("command_latency_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

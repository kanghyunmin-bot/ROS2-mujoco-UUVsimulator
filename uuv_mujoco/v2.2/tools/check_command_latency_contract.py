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
from bridge.sitl_arm_mode_queue_arm import queue_arm_command_impl  # noqa: E402
from bridge.sitl_arm_mode_service_send import send_pending_arm_to_any_link  # noqa: E402
from bridge.sitl_vehicle_heartbeat_filter import _heartbeat_is_vehicle  # noqa: E402
from bridge.ros2_rc_override_forward_cache import (  # noqa: E402
    clear_pending_rc_override,
    service_pending_rc_override_forward,
    store_pending_rc_override,
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
    _assert(ARM_PENDING_RESEND_PERIOD_S >= 0.20, "arm resend must not flood MAVLink command ACKs")
    _assert(MODE_PENDING_RESEND_PERIOD_S >= 0.20, "mode resend must not flood MAVLink command ACKs")
    _assert(ARM_PENDING_RESEND_PERIOD_S <= 0.50, "arm resend should still recover within 500 ms")
    _assert(MODE_PENDING_RESEND_PERIOD_S <= 0.50, "mode resend should still recover within 500 ms")


def check_command_subscriber_depths() -> None:
    by_topic = {spec.topic: spec.qsize for spec in SUBSCRIBER_SPECS}
    for topic in COMMAND_TOPICS & by_topic.keys():
        _assert(by_topic[topic] == 1, f"{topic} subscriber depth must be 1")


class _ArmRetryFakeTransport:
    def __init__(self) -> None:
        self._sitl_cmd_debug = False
        self._sitl_pending_arm_neutral_sent = False
        self._sitl_pending_arm_target = None
        self._sitl_pending_arm_start_wall = -1.0
        self._sitl_pending_arm_last_send_wall = -1.0
        self._sitl_pending_arm_reached_after_wall = -1.0
        self._sitl_vehicle_armed = False
        self.arm_sends: list[tuple[str, bool]] = []
        self.neutral_sends: list[str] = []
        self.service_calls = 0

    def _mavs_for_arm_mode_commands(self) -> list[str]:
        return ["servo-link", "command-link"]

    def _resolve_mav_target(self, mav: str) -> tuple[int, int]:
        return (1, 1)

    def _send_gcs_heartbeat(self, *, force: bool, mav: str) -> None:
        _assert(force, "arm resend must force GCS heartbeat")

    def _send_arm_disarm_mavlink(
        self,
        mav: str,
        target_sys: int,
        target_comp: int,
        target_arm: bool,
        *,
        force: bool,
    ) -> bool:
        _assert((target_sys, target_comp, force) == (1, 1, True), "arm send target contract changed")
        self.arm_sends.append((mav, bool(target_arm)))
        return True

    def _neutral_rc_values(self) -> list[int]:
        return [1500] * 18

    def _send_rc_channels_override(self, mav: str, target_sys: int, target_comp: int, values: list[int]) -> None:
        _assert((target_sys, target_comp) == (1, 1), "neutral RC target contract changed")
        _assert(values == [1500] * 18, "arm neutral RC values changed")
        self.neutral_sends.append(mav)

    def _service_pending_arm_command(self, now_wall: float) -> None:
        self.service_calls += 1


class _HeartbeatMavlink:
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3


class _HeartbeatMavutil:
    mavlink = _HeartbeatMavlink()


class _HeartbeatFakeTransport:
    def __init__(self) -> None:
        self._sitl_mavutil = _HeartbeatMavutil()
        self._sitl_mavlink_target_sysid = 1
        self._sitl_mavlink_target_compid = 0


class _HeartbeatMsg:
    def __init__(self, *, src_sys: int, src_comp: int, autopilot: int) -> None:
        self.autopilot = autopilot
        self._src_sys = int(src_sys)
        self._src_comp = int(src_comp)

    def get_type(self) -> str:
        return "HEARTBEAT"

    def get_srcSystem(self) -> int:
        return self._src_sys

    def get_srcComponent(self) -> int:
        return self._src_comp


class _PendingRcBridge:
    def __init__(self) -> None:
        import threading

        self._mavros_rc_override_cache_lock = threading.RLock()
        self._mavros_pending_rc_override_channels = None
        self._mavros_pending_rc_override_wall = -1.0
        self._mavros_last_forwarded_rc_override_wall = -1.0
        self._mavros_rc_override_forward_period_s = 0.0
        self._mavros_rc_override_stale_s = 3.0
        self.forwarded: list[list[int]] = []


def check_vehicle_heartbeat_requires_ardupilot_autopilot_type() -> None:
    transport = _HeartbeatFakeTransport()
    _assert(
        _heartbeat_is_vehicle(
            transport,
            _HeartbeatMsg(src_sys=1, src_comp=1, autopilot=_HeartbeatMavlink.MAV_AUTOPILOT_ARDUPILOTMEGA),
        ),
        "ArduPilot heartbeat from target system must be accepted",
    )
    _assert(
        not _heartbeat_is_vehicle(transport, _HeartbeatMsg(src_sys=1, src_comp=191, autopilot=8)),
        "target-sysid non-ArduPilot heartbeat must not overwrite vehicle state",
    )


def check_arm_retry_neutral_rc_only_initial() -> None:
    transport = _ArmRetryFakeTransport()
    _assert(send_pending_arm_to_any_link(transport, True), "initial arm send failed")
    _assert(send_pending_arm_to_any_link(transport, True), "arm retry send failed")
    _assert(
        transport.neutral_sends == ["servo-link", "command-link"],
        "arm retry must not keep overwriting pilot RC with neutral RC",
    )
    _assert(
        transport.arm_sends
        == [
            ("servo-link", True),
            ("command-link", True),
            ("servo-link", True),
            ("command-link", True),
        ],
        "arm command should still retry on every pending service interval",
    )

    transport._sitl_pending_arm_neutral_sent = True
    _assert(queue_arm_command_impl(transport, True), "new arm target should queue")
    _assert(
        transport._sitl_pending_arm_neutral_sent is False,
        "new arm queue must allow one fresh neutral RC priming frame",
    )


def check_pending_rc_override_clears_after_forward() -> None:
    import bridge.ros2_rc_override_forward_cache as cache_module

    bridge = _PendingRcBridge()
    frame = [1500] * 18
    frame[4] = 1650
    store_pending_rc_override(bridge, frame)
    _assert(bridge._mavros_pending_rc_override_channels is not None, "pending RC frame was not stored")

    original_forward = cache_module.forward_rc_override_to_sitl
    try:
        cache_module.forward_rc_override_to_sitl = lambda owner, channels: owner.forwarded.append(list(channels)) or True
        service_pending_rc_override_forward(bridge)
    finally:
        cache_module.forward_rc_override_to_sitl = original_forward

    _assert(bridge.forwarded == [frame], "pending RC frame should forward once")
    _assert(bridge._mavros_pending_rc_override_channels is None, "successful pending RC forward must clear stale cache")
    _assert(bridge._mavros_pending_rc_override_wall == -1.0, "successful pending RC forward must reset pending timestamp")

    store_pending_rc_override(bridge, frame)
    clear_pending_rc_override(bridge)
    _assert(bridge._mavros_pending_rc_override_channels is None, "explicit pending RC clear must drop cached frame")


def main() -> int:
    check_resend_periods()
    check_command_subscriber_depths()
    check_vehicle_heartbeat_requires_ardupilot_autopilot_type()
    check_arm_retry_neutral_rc_only_initial()
    check_pending_rc_override_clears_after_forward()
    print("command_latency_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

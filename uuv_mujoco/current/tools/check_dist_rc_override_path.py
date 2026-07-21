#!/usr/bin/env python3
"""Regression check for the dist-style RC override forwarding path."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.sitl_rc_override_send import send_rc_override  # noqa: E402
from sim.transport.mavlink_rc_override_sender import send_rc_channels_override_on_link  # noqa: E402


class FakeMavSender:
    def __init__(self) -> None:
        self.sent: list[tuple[int, int, tuple[int, ...]]] = []

    def rc_channels_override_send(self, target_sys: int, target_comp: int, *values: int) -> None:
        self.sent.append((int(target_sys), int(target_comp), tuple(int(v) for v in values)))


class FakeMav:
    def __init__(self) -> None:
        self.mav = FakeMavSender()


class FakeTransport:
    def __init__(self, *, dedicated_command_link: bool = True) -> None:
        self._sitl_mav = FakeMav()
        self._sitl_cmd_mav = FakeMav() if dedicated_command_link else None
        self._sitl_cmd_debug = False
        self._sitl_last_external_rc_override_wall = -1.0
        self._sitl_last_rc_override_values: list[int] = []
        self._sitl_last_rc_override_values_wall = -1.0
        self._sensor_replay_rc_seen = False
        self._sensor_replay_immediate_last_frame_count = 7
        self._sitl_last_rc_override_log_wall = -1.0
        self.warnings: list[str] = []

    def _mav_for_commands(self):
        return self._sitl_cmd_mav if self._sitl_cmd_mav is not None else self._sitl_mav

    def _resolve_mav_target(self, mav=None):
        if mav is self._sitl_cmd_mav:
            return 1, 190
        if mav is self._sitl_mav:
            return 1, 1
        raise AssertionError("RC override used an unknown MAVLink link")

    def _send_rc_channels_override(self, mav, target_sys: int, target_comp: int, values: list[int]) -> int:
        mav.mav.rc_channels_override_send(
            int(target_sys),
            int(target_comp),
            *(int(v) for v in values[:8]),
        )
        return 8

    def _command_link_for_mav(self, mav):
        return object()

    def _warn_rc_override_not_forwarded(self, reason: str) -> None:
        self.warnings.append(str(reason))


class NoCommandLinkTransport(FakeTransport):
    def _send_rc_channels_override(self, mav, target_sys: int, target_comp: int, values: list[int]) -> int:
        return 0


class NoTargetTransport(FakeTransport):
    def _resolve_mav_target(self, mav=None):
        return None


def _sent_on(mav: FakeMav) -> list[tuple[int, int, tuple[int, ...]]]:
    return mav.mav.sent


def main() -> int:
    low_level_mav = FakeMav()
    low_level_count = send_rc_channels_override_on_link(
        low_level_mav,
        1,
        1,
        [1500, 1500, 1650, 1520, 1500, 1500, 1500, 1500, 1900, 1800],
    )
    if low_level_count != 8:
        raise AssertionError(f"low-level RC override should send 8 channels, got {low_level_count}")
    if _sent_on(low_level_mav) != [(1, 1, (1500, 1500, 1650, 1520, 1500, 1500, 1500, 1500))]:
        raise AssertionError(f"low-level RC override did not use dist 8-channel payload: {_sent_on(low_level_mav)}")

    transport = FakeTransport()
    frame = [1500, 1500, 1650, 1520, 1500, 1500, 1500, 1500, 1900, 1800]
    if not send_rc_override(transport, frame):
        raise AssertionError("RC override send failed")

    command_sent = _sent_on(transport._sitl_cmd_mav)
    if command_sent != [(1, 190, tuple(frame[:8]))]:
        raise AssertionError(f"unexpected command-link RC frame: {command_sent}")
    if _sent_on(transport._sitl_mav):
        raise AssertionError("RC override should prefer the dedicated command link")
    if transport._sitl_last_rc_override_values != frame[:8]:
        raise AssertionError("last RC override values should be the 8-channel dist frame")
    if not transport._sensor_replay_rc_seen:
        raise AssertionError("RC seen marker was not set")
    if transport._sensor_replay_immediate_last_frame_count is not None:
        raise AssertionError("sensor replay immediate frame count should be cleared")
    if transport.warnings:
        raise AssertionError(f"unexpected warnings: {transport.warnings}")

    fallback_transport = FakeTransport(dedicated_command_link=False)
    if not send_rc_override(fallback_transport, frame):
        raise AssertionError("fallback RC override send failed")
    if _sent_on(fallback_transport._sitl_mav) != [(1, 1, tuple(frame[:8]))]:
        raise AssertionError(f"fallback did not use primary link: {_sent_on(fallback_transport._sitl_mav)}")

    short_transport = FakeTransport()
    if not send_rc_override(short_transport, [1600, 1400]):
        raise AssertionError("short RC override send failed")
    expected_short = (1600, 1400, 65535, 65535, 65535, 65535, 65535, 65535)
    if _sent_on(short_transport._sitl_cmd_mav) != [(1, 190, expected_short)]:
        raise AssertionError(f"short RC frame was not dist-padded: {_sent_on(short_transport._sitl_cmd_mav)}")

    no_target = NoTargetTransport()
    if send_rc_override(no_target, frame):
        raise AssertionError("RC override should fail before a target heartbeat is available")
    if not no_target.warnings:
        raise AssertionError("missing warning for unresolved target")

    no_link = NoCommandLinkTransport()
    if send_rc_override(no_link, frame):
        raise AssertionError("RC override should fail when command send returns zero channels")
    if not no_link.warnings:
        raise AssertionError("missing warning for unavailable command link")

    print("dist_rc_override_path=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

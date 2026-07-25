#!/usr/bin/env python3
"""Regression smoke for replay RCOUT plant-input callback ownership."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_replay_rcout import _handle_replay_rcout_channels, _on_replay_rcout_override  # noqa: E402


class FakeLock:
    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        return False


class FakeTransport:
    def __init__(self) -> None:
        self.injected: list[tuple[list[int], float, str]] = []

    def inject_servo_pwm_values(self, channels: list[int], *, hold_s: float, source: str) -> None:
        self.injected.append((list(channels), float(hold_s), source))


class FakeBridge:
    def __init__(self, *, with_transport: bool = True) -> None:
        self._sitl_transport = FakeTransport() if with_transport else None
        self._sitl_transport_lock = FakeLock()
        self._replay_rcout_handler = None
        self._replay_rcout_count = 0
        self._replay_rcout_last_log_wall = -10.0
        self._mavros_last_rc_override_warn_wall = -10.0

    def _handle_replay_rcout_channels(self, channels: list[int], *, source: str) -> None:
        _handle_replay_rcout_channels(self, channels, source=source)


def check_transport_injection() -> None:
    bridge = FakeBridge()
    channels = list(range(1500, 1510))
    _handle_replay_rcout_channels(bridge, channels, source="unit")
    assert bridge._replay_rcout_count == 1
    assert bridge._sitl_transport.injected == [(list(range(1500, 1508)), 1.0, "unit")]


def check_direct_handler_fallback() -> None:
    bridge = FakeBridge(with_transport=False)
    calls: list[list[int]] = []
    bridge._replay_rcout_handler = lambda channels: calls.append(list(channels))
    _on_replay_rcout_override(bridge, SimpleNamespace(channels=list(range(1600, 1610))))
    assert bridge._replay_rcout_count == 1
    assert calls == [list(range(1600, 1608))]


def check_short_frame_is_ignored() -> None:
    bridge = FakeBridge()
    _handle_replay_rcout_channels(bridge, [1500, 1501], source="short")
    assert bridge._replay_rcout_count == 0
    assert bridge._sitl_transport.injected == []


def main() -> int:
    check_transport_injection()
    check_direct_handler_fallback()
    check_short_frame_is_ignored()
    print("ros2_replay_rcout=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

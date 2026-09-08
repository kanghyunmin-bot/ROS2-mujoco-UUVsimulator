#!/usr/bin/env python3
"""Dependency-light checks for the GUI neutral-then-release sequence."""

from __future__ import annotations

from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


class _OverrideRCIn:
    CHAN_RELEASE = 0
    CHAN_NOCHANGE = 65535

    def __init__(self, kind: str = "") -> None:
        self.kind = kind
        self.channels: list[int] = []


def _make_rc_override_message(_layout, **axes: float) -> _OverrideRCIn:
    kind = "neutral" if all(abs(float(value)) <= 1.0e-12 for value in axes.values()) else "active"
    return _OverrideRCIn(kind)


helpers_stub = ModuleType("gui.helpers")
helpers_stub.make_rc_override_message = _make_rc_override_message
helpers_stub.make_rc_release_message = lambda: _OverrideRCIn("release")
helpers_stub.sanitize_primary_rc_override_channels = lambda channels: list(channels)
runtime_stub = ModuleType("gui.runtime")
runtime_stub.OverrideRCIn = _OverrideRCIn
sys.modules["gui.helpers"] = helpers_stub
sys.modules["gui.runtime"] = runtime_stub

from gui import node_rc_override_publishers as publishers  # noqa: E402


class _Publisher:
    def __init__(self) -> None:
        self.kinds: list[str] = []

    def publish(self, message: _OverrideRCIn) -> None:
        self.kinds.append(message.kind)


class _Owner:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._snapshot = SimpleNamespace(mode="MANUAL")
        self._rc_override_publisher_lock = threading.RLock()
        self._rc_override_pub = _Publisher()
        self._rc_override_burst_count = 1
        self._rc_override_release_delay_s = 0.15
        self._rc_override_publish_generation = 0
        self.scheduled: list[tuple[float, object]] = []

    def _active_layout(self):
        return None

    def _schedule_once(self, delay_s: float, callback) -> None:
        self.scheduled.append((float(delay_s), callback))


def check_neutral_precedes_delayed_release() -> None:
    owner = _Owner()

    publishers.publish_rc_neutral_then_release(owner)

    assert owner._rc_override_pub.kinds == ["neutral"]
    assert len(owner.scheduled) == 1
    delay_s, callback = owner.scheduled.pop()
    assert delay_s == 0.15
    callback()
    assert owner._rc_override_pub.kinds == ["neutral", "release"]


def check_new_pilot_input_cancels_stale_release() -> None:
    owner = _Owner()

    publishers.publish_rc_neutral_then_release(owner)
    _, delayed_release = owner.scheduled.pop()
    publishers.publish_rc_override(
        owner,
        yaw=0.0,
        heave=0.0,
        forward=0.2,
        lateral=0.0,
    )
    delayed_release()

    assert owner._rc_override_pub.kinds == ["neutral", "active"]
    assert "release" not in owner._rc_override_pub.kinds


def main() -> int:
    check_neutral_precedes_delayed_release()
    check_new_pilot_input_cancels_stale_release()
    print("gui_rc_release_sequence=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

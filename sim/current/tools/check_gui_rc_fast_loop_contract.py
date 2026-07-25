#!/usr/bin/env python3
"""Regression checks for the GUI RC fast-publish loop."""

from __future__ import annotations

from pathlib import Path
import sys
from types import ModuleType


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

runtime_stub = ModuleType("gui.runtime")
runtime_stub.rclpy = type("FakeRclpy", (), {"ok": staticmethod(lambda: False), "shutdown": staticmethod(lambda: None)})()
sys.modules.setdefault("gui.runtime", runtime_stub)

from gui.app_lifecycle import _update_rc_fast, _schedule_rc_fast_update  # noqa: E402
from gui.config import RC_FAST_PERIOD_MS  # noqa: E402
from gui.models import ControlCommands  # noqa: E402


class FakeRoot:
    def __init__(self) -> None:
        self.after_calls: list[tuple[int, object]] = []

    def winfo_exists(self) -> bool:
        return True

    def after(self, delay_ms: int, callback) -> str:
        self.after_calls.append((int(delay_ms), callback))
        return f"after-{len(self.after_calls)}"


class FakeOwner:
    def __init__(self) -> None:
        self.root = FakeRoot()
        self._closed = False
        self._rc_fast_after_id = None
        self.publish_calls = 0
        self._update_rc_fast = lambda: None

    def _read_control_commands(self) -> ControlCommands:
        return ControlCommands(
            velocity_forward=0.0,
            velocity_lateral=0.0,
            velocity_heave=0.0,
            velocity_yaw=0.0,
            rc_forward=0.2,
            rc_lateral=0.0,
            rc_heave=0.0,
            rc_yaw=0.0,
        )

    def _publish_active_controls(self, _commands: ControlCommands) -> None:
        self.publish_calls += 1

    def _schedule_rc_fast_update(self) -> None:
        _schedule_rc_fast_update(self)


def main() -> int:
    if RC_FAST_PERIOD_MS > 5:
        raise AssertionError(f"RC fast loop must be <=5ms, got {RC_FAST_PERIOD_MS}ms")

    owner = FakeOwner()
    _update_rc_fast(owner)
    if owner.publish_calls != 1:
        raise AssertionError(f"expected one RC publish, got {owner.publish_calls}")
    if owner.root.after_calls != [(RC_FAST_PERIOD_MS, owner._update_rc_fast)]:
        raise AssertionError(f"bad RC fast schedule: {owner.root.after_calls!r}")
    if owner._rc_fast_after_id != "after-1":
        raise AssertionError(f"RC fast after id not stored: {owner._rc_fast_after_id!r}")

    closed = FakeOwner()
    closed._closed = True
    _update_rc_fast(closed)
    if closed.publish_calls != 0 or closed.root.after_calls:
        raise AssertionError("closed GUI must not publish or reschedule RC fast loop")

    print("gui_rc_fast_loop_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

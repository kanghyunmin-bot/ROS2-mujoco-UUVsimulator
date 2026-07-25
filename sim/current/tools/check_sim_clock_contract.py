#!/usr/bin/env python3
"""Verify that MuJoCo time is the exact ROS /clock and sensor stamp source."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import math
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_bridge_publish_stamp import acquire_ros_stamp


class FakeClock:
    def __init__(self) -> None:
        self.clock = SimpleNamespace(sec=0, nanosec=0)


class FakeBridge:
    Clock = FakeClock
    pub_clock = object()
    pub_uuv_mujoco_clock = object()

    def __init__(self) -> None:
        self._ros_error_reported = False
        self._ros_ok = True
        self.published = []

    def _safe_publish(self, publisher, msg, label: str) -> bool:
        self.published.append((publisher, msg, label))
        return True


def main() -> int:
    bridge = FakeBridge()
    stamp = acquire_ros_stamp(bridge, 12.345678901)
    assert stamp is not None
    assert stamp.sec == 12
    assert stamp.nanosec == 345_678_901
    assert len(bridge.published) == 2
    assert bridge.published[0][2] == "/clock"
    assert bridge.published[0][1].clock is stamp
    assert bridge.published[1][2] == "/uuv_mujoco/clock"
    assert bridge.published[1][1] is bridge.published[0][1]

    for bad_time in (-0.1, math.inf, math.nan):
        failed_bridge = FakeBridge()
        assert acquire_ros_stamp(failed_bridge, bad_time) is None
        assert not failed_bridge._ros_ok

    print("sim_clock_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

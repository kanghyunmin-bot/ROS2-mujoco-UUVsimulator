#!/usr/bin/env python3
"""Regression smoke for raw SITL JSON servo runtime state."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.runtime.sitl_servo_state import SitlServoRuntime  # noqa: E402


def check_packet_and_target_application() -> None:
    runtime = SitlServoRuntime.create(
        all_thruster_names=["t1", "t2"],
        raw_map=["t1", "t2"],
        servo_signs=[1.0, -1.0],
        sitl_servo_scale=0.5,
        timeout_s=0.8,
    )
    runtime.on_packet([1900, 1100])
    assert runtime.pwm_values[:2] == [1900, 1100]
    assert runtime.cmd_norm == {"t1": 1.0, "t2": 1.0}
    targets = {"t1": 0.0, "t2": 0.0}
    stale = runtime.apply_to_targets(targets, now_wall=runtime.last_wall["value"], timeout_s=0.8)
    assert stale is False
    assert targets == {"t1": 0.5, "t2": 0.5}
    assert runtime.mapping_label() == "ch1->t1*+1, ch2->t2*-1"


def check_timeout_clears_targets() -> None:
    runtime = SitlServoRuntime.create(
        all_thruster_names=["t1", "t2"],
        raw_map=["t1", "t2"],
        servo_signs=[1.0, 1.0],
        sitl_servo_scale=1.0,
        timeout_s=0.8,
    )
    targets = {"t1": 0.25, "t2": -0.25}
    stale = runtime.apply_to_targets(targets, now_wall=100.0, timeout_s=0.8)
    assert stale is True
    assert targets == {"t1": 0.0, "t2": 0.0}


def main() -> int:
    check_packet_and_target_application()
    check_timeout_clears_targets()
    print("sitl_servo_runtime=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

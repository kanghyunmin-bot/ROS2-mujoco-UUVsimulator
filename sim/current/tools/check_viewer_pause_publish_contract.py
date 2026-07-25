#!/usr/bin/env python3
"""Ensure viewer pause keeps ROS sensor publication alive at its cadence."""

from __future__ import annotations

from pathlib import Path
import sys


CURRENT_DIR = Path(__file__).resolve().parents[1]
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from sim.runtime.simulation_loop_cadence import ViewerLoopCadence  # noqa: E402
from sim.runtime.simulation_loop_clocks import ViewerLoopClocks, run_paused_step  # noqa: E402


def main() -> int:
    calls: list[tuple[bool, bool]] = []

    def run_step(paused: bool, publish_ros: bool):
        calls.append((paused, publish_ros))
        return (0.0, 0.0, 0.0, 0.0)

    cadence = ViewerLoopCadence(
        target_dt=0.005,
        sensor_dt=0.1,
        viewer_dt=0.05,
        max_catchup_steps=4,
        max_sensor_catchup=4,
        max_step_lag_s=0.25,
        max_sensor_lag_s=0.25,
        max_sleep_s=0.001,
    )
    clocks = ViewerLoopClocks(next_step_wall=1.0, next_sensor_wall=1.0, next_viewer_wall=1.0)
    for now in (1.0, 1.04, 1.10, 1.35):
        run_paused_step(run_step=run_step, now_wall=now, cadence=cadence, clocks=clocks)
    expected = [(True, True), (True, False), (True, True), (True, True)]
    if calls != expected:
        raise AssertionError(f"paused publish cadence mismatch: {calls}")
    if not (1.35 < clocks.next_sensor_wall <= 1.45):
        raise AssertionError(f"paused sensor deadline was not advanced past wall time: {clocks.next_sensor_wall}")
    if abs(clocks.next_step_wall - 1.355) > 1.0e-12:
        raise AssertionError(f"paused physics deadline mismatch: {clocks.next_step_wall}")

    print("viewer_pause_publish_contract=PASS paused_calls=4 publishes=3")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

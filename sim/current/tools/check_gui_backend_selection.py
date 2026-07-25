#!/usr/bin/env python3
"""Regression checks for GUI backend scoring/selection policy."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
PARENT = ROOT.parent
if str(PARENT) not in sys.path:
    sys.path.insert(0, str(PARENT))

from gui.config import BACKEND_MAVROS, BACKEND_NONE, BACKEND_SIM_BRIDGE, DEFAULT_AUTO_BACKEND  # noqa: E402
from gui.node_backend_selection import mavros_score, select_backend, sim_bridge_score  # noqa: E402


COUNT_KEYS = (
    "vehicle_info_services",
    "arm_services",
    "mode_services",
    "rc_out_publishers",
    "state_publishers",
    "pose_publishers",
    "velocity_body_publishers",
    "rc_in_publishers",
    "velocity_local_publishers",
    "bridge_imu_publishers",
    "bridge_rovio_publishers",
    "bridge_dvl_odom_publishers",
    "bridge_dvl_velocity_publishers",
    "bridge_depth_publishers",
    "bridge_battery_publishers",
    "rc_override_subscribers",
    "manual_control_subscribers",
)


def counts(**updates: int) -> dict[str, int]:
    data = {key: 0 for key in COUNT_KEYS}
    data.update(updates)
    return data


def main() -> int:
    mav_counts = counts(vehicle_info_services=1, rc_out_publishers=1, velocity_body_publishers=1)
    assert mavros_score(mav_counts) == 9
    assert select_backend(BACKEND_NONE, mavros_score(mav_counts), sim_bridge_score(mav_counts), mav_counts) == BACKEND_MAVROS

    sim_counts = counts(rc_in_publishers=1, bridge_rovio_publishers=1, rc_override_subscribers=1)
    assert sim_bridge_score(sim_counts) == 8
    assert select_backend(BACKEND_NONE, mavros_score(sim_counts), sim_bridge_score(sim_counts), sim_counts) == BACKEND_SIM_BRIDGE

    tie_counts = counts(rc_out_publishers=1, rc_in_publishers=1)
    assert select_backend(BACKEND_NONE, 3, 3, tie_counts) == BACKEND_MAVROS
    assert select_backend(BACKEND_NONE, 0, 0, counts()) == BACKEND_NONE
    assert select_backend(BACKEND_SIM_BRIDGE, 0, 0, counts()) == DEFAULT_AUTO_BACKEND
    print("gui_backend_selection=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

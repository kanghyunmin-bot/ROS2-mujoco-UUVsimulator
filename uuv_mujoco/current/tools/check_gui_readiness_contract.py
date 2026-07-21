#!/usr/bin/env python3
"""Regression checks for GUI arm/mode readiness gate policy."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
PARENT = ROOT.parent
if str(PARENT) not in sys.path:
    sys.path.insert(0, str(PARENT))

from gui.config import BACKEND_SIM_BRIDGE  # noqa: E402
from gui.models import TelemetrySnapshot  # noqa: E402
from gui.readiness_contract import arm_mode_gate_reason, sitl_extnav_ready, sitl_mavlink_command_alive  # noqa: E402


def ready_snapshot() -> TelemetrySnapshot:
    return TelemetrySnapshot(
        connected=True,
        state_age_s=0.1,
        depth_age_s=0.1,
        imu_age_s=0.1,
        sitl_mavlink_active=True,
        sitl_mavlink_status_age_s=0.1,
        sitl_mavlink_heartbeat_age_s=0.1,
        sitl_mavlink_command_heartbeat_age_s=0.1,
        sitl_mavlink_rc_override_ready=True,
        sitl_extnav_required=True,
        sitl_extnav_ready=True,
    )


def assert_reason(name: str, snap: TelemetrySnapshot, expected: str, **kwargs: object) -> None:
    actual = arm_mode_gate_reason(
        BACKEND_SIM_BRIDGE,
        snap,
        settle_left_s=float(kwargs.pop("settle_left_s", 0.0)),
        **kwargs,
    )
    assert actual == expected, f"{name}: expected {expected!r}, got {actual!r}"


def check_command_alive() -> None:
    snap = ready_snapshot()
    assert sitl_mavlink_command_alive(BACKEND_SIM_BRIDGE, snap)
    snap.sitl_mavlink_command_heartbeat_age_s = float("inf")
    assert sitl_mavlink_command_alive(BACKEND_SIM_BRIDGE, snap)
    snap.sitl_mavlink_heartbeat_age_s = 4.0
    assert sitl_mavlink_command_alive(BACKEND_SIM_BRIDGE, snap)
    snap.sitl_mavlink_rc_override_ready = False
    assert not sitl_mavlink_command_alive(BACKEND_SIM_BRIDGE, snap)


def check_extnav_ready() -> None:
    snap = ready_snapshot()
    assert sitl_extnav_ready(BACKEND_SIM_BRIDGE, snap)
    snap.sitl_extnav_ready = False
    assert not sitl_extnav_ready(BACKEND_SIM_BRIDGE, snap)
    snap.sitl_extnav_required = False
    assert sitl_extnav_ready(BACKEND_SIM_BRIDGE, snap)


def check_arm_mode_gate_order() -> None:
    assert_reason("disarm never blocked", TelemetrySnapshot(), "", arm_value=False)
    assert_reason("manual no-op mode not blocked", TelemetrySnapshot(), "", mode="MANUAL")

    snap = ready_snapshot()
    assert_reason("ready arm", snap, "", arm_value=True)

    stale_state = ready_snapshot()
    stale_state.state_age_s = 3.0
    assert_reason("stale state with live command link", stale_state, "", arm_value=True)
    stale_state.sitl_mavlink_active = False
    stale_state.sitl_mavlink_heartbeat_age_s = 4.0
    stale_state.sitl_mavlink_rc_override_ready = False
    assert_reason("stale state and dead command link", stale_state, "waiting for fresh vehicle state", arm_value=True)

    no_depth = ready_snapshot()
    no_depth.depth_age_s = float("inf")
    assert_reason("no depth", no_depth, "waiting for Bar30/depth feedback", arm_value=True)

    stale_depth = ready_snapshot()
    stale_depth.depth_age_s = 3.0
    assert_reason("stale depth", stale_depth, "waiting for fresh Bar30/depth feedback", arm_value=True)

    no_imu = ready_snapshot()
    no_imu.imu_age_s = float("inf")
    assert_reason("no imu", no_imu, "waiting for IMU feedback", arm_value=True)

    no_mav = ready_snapshot()
    no_mav.sitl_mavlink_active = False
    assert_reason("no mavlink", no_mav, "waiting for SITL MAVLink command link", arm_value=True)

    assert_reason(
        "settle",
        ready_snapshot(),
        "waiting EKF/ExternalNav settle (1.5s left)",
        arm_value=True,
        settle_left_s=1.5,
    )


def main() -> int:
    check_command_alive()
    check_extnav_ready()
    check_arm_mode_gate_order()
    print("gui_readiness_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

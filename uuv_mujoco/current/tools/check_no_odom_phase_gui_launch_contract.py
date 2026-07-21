#!/usr/bin/env python3
"""Offline contract check for the explicit NO_ODOM_PHASE GUI/launch path."""

from __future__ import annotations

import ast
from pathlib import Path


RUNTIME_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = Path(__file__).resolve().parents[3]
SIM_GUI = RUNTIME_ROOT / "gui"
PINGER = (
    WORKSPACE
    / "rospkg"
    / "src"
    / "kmu26_control_packages"
    / "kmu26_pinger_homing"
)
REAL_GUI = WORKSPACE / "rospkg" / "src" / "kmu26_auv_web_gui"
PHASE_ESTIMATOR = (
    WORKSPACE
    / "rospkg"
    / "src"
    / "kmu26_auv_hydrophone"
    / "audio_capture"
    / "src"
    / "audio_phase_estimator.cpp"
)


def _read(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".py":
        ast.parse(text, filename=str(path))
    return text


def main() -> int:
    launch = _read(PINGER / "launch" / "pinger_homing_real.launch.py")
    manager = _read(SIM_GUI / "web_process_manager.py")
    sim_html = _read(SIM_GUI / "web_static" / "index.html")
    sim_js = _read(SIM_GUI / "web_static" / "app.js")
    real_server = _read(REAL_GUI / "kmu26_auv_web_gui" / "server.py")
    real_ros = _read(REAL_GUI / "kmu26_auv_web_gui" / "ros_interface.py")
    real_html = _read(REAL_GUI / "web" / "index.html")
    real_js = _read(REAL_GUI / "web" / "app.js")
    phase_estimator = _read(PHASE_ESTIMATOR)

    assert 'choices=["odometry", "no_odom_phase"]' in launch
    assert '"navigation_mode": navigation_mode' in launch
    assert launch.count("/pinger_homing/disabled/odometry") >= 4
    assert '"imu_topic": imu_topic' in launch
    assert '"depth_pose_topic": depth_topic' in launch
    assert 'LaunchConfiguration("no_odom_probe_leg_s")' in launch
    assert 'LaunchConfiguration("no_odom_forward_duration_s")' in launch
    assert 'LaunchConfiguration("no_odom_vertical_control_enabled")' in launch
    assert launch.count('"audio_topic": audio_topic') >= 2
    assert launch.count('"use_odometry": ParameterValue(') == 2
    assert '"/audio_boosted"' not in launch

    # The controller consumes the estimator's already-computed phase
    # observations. Keep this output-only interface overlay explicit so a
    # clean upstream clone cannot build successfully but remain audio-stale.
    assert '"/audio_phase_estimator/delta_range_m"' in phase_estimator
    assert '"/audio_phase_estimator/iq_magnitude"' in phase_estimator
    assert "publish_phase_debug(" in phase_estimator
    assert 'declare_parameter<bool>("use_odometry", true)' in phase_estimator
    assert "if (use_odometry_)" in phase_estimator
    assert "if (publish_homing_direction_)" in phase_estimator

    assert '{"phase", "no_odom_phase", "snr"}' in manager
    assert 'f"navigation_mode:={requested_navigation_mode}"' in manager
    assert '"odometry_topic:=/pinger_homing/disabled/odometry"' in manager
    assert '"imu_topic:=/mavros/imu/data"' in manager
    assert 'mode_label = (' in manager and '"NO_ODOM_PHASE"' in manager

    for html in (sim_html, real_html):
        assert 'value="phase"' in html
        assert 'value="no_odom_phase"' in html
        assert 'value="snr"' in html
        assert "NO_ODOM_PHASE" in html
    for javascript in (sim_js, real_js):
        assert '"no_odom_phase" ? "no_odom_phase" : "odometry"' in javascript
        assert "/odometry/filtered disabled" in javascript

    assert 'navigation_mode not in {"odometry", "no_odom_phase"}' in real_server
    assert '"/pinger_homing/disabled/odometry"' in real_server
    assert 'detail="NO_ODOM_PHASE requires estimator_mode=phase"' in real_server
    assert '"navigation_mode": (' in real_ros
    assert '"odometry_required": (' in real_ros
    assert '"imu_fresh":' in real_ros
    assert '"depth_fresh": (' in real_ros

    print("no_odom_phase_gui_launch_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Regression checks for RC override frame marker/PWM contracts."""

from __future__ import annotations

import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
PARENT = ROOT.parent
if str(PARENT) not in sys.path:
    sys.path.insert(0, str(PARENT))

from sim.contracts import (  # noqa: E402
    PWM_CENTER,
    RC_EXTENSION_NO_CHANGE_VALUE,
    RC_IGNORE_VALUE,
    RC_OVERRIDE_CHANNEL_COUNT,
    RC_RELEASE_VALUE,
    neutral_rc_override_frame,
    normalize_ardusub_rc_override,
    sanitize_primary_rc,
)
from axis_rc_contract import AXIS_TO_CHANNEL  # noqa: E402


def check_gui_rc_input_span_contract() -> None:
    os.environ.pop("UUV_GUI_RC_PWM_SPAN", None)
    os.environ.pop("ROS2_UUV_MAVROS_RC_PWM_SPAN", None)

    from gui.config import RC_PWM_SPAN  # noqa: PLC0415
    from gui.gui_rc_pwm import axis_to_pwm  # noqa: PLC0415
    from gui.sim_stack_env_forced_rc import rc_override_contract  # noqa: PLC0415
    from bridge.ros2_bridge_config_mavros import configure_mavros_rc_contract  # noqa: PLC0415

    assert RC_PWM_SPAN == 400.0
    assert axis_to_pwm(1.0) == 1900
    assert axis_to_pwm(-1.0) == 1100
    forced = rc_override_contract()
    assert forced["UUV_GUI_RC_PWM_SPAN"] == "400"
    assert forced["ROS2_UUV_MAVROS_RC_PWM_SPAN"] == "400"

    class Bridge:
        pass

    bridge = Bridge()
    configure_mavros_rc_contract(bridge)
    assert bridge._mavros_rc_pwm_span == 400.0


def check_neutral_frame() -> None:
    frame = neutral_rc_override_frame()
    assert len(frame) == RC_OVERRIDE_CHANNEL_COUNT
    assert frame[:8] == [PWM_CENTER] * 8
    assert frame[8:] == [RC_RELEASE_VALUE] * 10


def check_sanitize_primary_rc() -> None:
    raw = [
        0,
        799,
        800,
        1500,
        2200,
        2201,
        RC_RELEASE_VALUE,
        RC_IGNORE_VALUE,
        RC_RELEASE_VALUE,
        RC_EXTENSION_NO_CHANGE_VALUE,
        RC_IGNORE_VALUE,
        799,
        800,
        1500,
        2200,
        2201,
        9999,
        -1,
    ]
    sanitized = sanitize_primary_rc(raw)
    assert sanitized == [
        PWM_CENTER,
        PWM_CENTER,
        800,
        1500,
        2200,
        PWM_CENTER,
        RC_RELEASE_VALUE,
        RC_IGNORE_VALUE,
        RC_RELEASE_VALUE,
        RC_EXTENSION_NO_CHANGE_VALUE,
        RC_IGNORE_VALUE,
        RC_RELEASE_VALUE,
        800,
        1500,
        2200,
        RC_RELEASE_VALUE,
        RC_RELEASE_VALUE,
        RC_RELEASE_VALUE,
    ]
    short = sanitize_primary_rc([1600, 1400], channel_count=10)
    assert short[:2] == [1600, 1400]
    assert short[2:8] == [PWM_CENTER] * 6
    assert short[8:] == [RC_RELEASE_VALUE] * 2


def check_normalize_ardusub_rc_override() -> None:
    raw = [
        0,
        RC_EXTENSION_NO_CHANGE_VALUE,
        RC_IGNORE_VALUE,
        799,
        800,
        1500,
        2200,
        2201,
        RC_RELEASE_VALUE,
        RC_EXTENSION_NO_CHANGE_VALUE,
        RC_IGNORE_VALUE,
        799,
        800,
        1500,
        2200,
        2201,
        9999,
        -1,
    ]
    normalized = normalize_ardusub_rc_override(raw)
    assert normalized == [
        RC_RELEASE_VALUE,
        RC_RELEASE_VALUE,
        RC_IGNORE_VALUE,
        RC_RELEASE_VALUE,
        800,
        1500,
        2200,
        RC_RELEASE_VALUE,
        RC_RELEASE_VALUE,
        RC_EXTENSION_NO_CHANGE_VALUE,
        RC_IGNORE_VALUE,
        RC_RELEASE_VALUE,
        800,
        1500,
        2200,
        RC_RELEASE_VALUE,
        RC_RELEASE_VALUE,
        RC_RELEASE_VALUE,
    ]
    short = normalize_ardusub_rc_override([1600, 0], channel_count=10)
    assert short == [
        1600,
        RC_RELEASE_VALUE,
        RC_IGNORE_VALUE,
        RC_IGNORE_VALUE,
        RC_IGNORE_VALUE,
        RC_IGNORE_VALUE,
        RC_IGNORE_VALUE,
        RC_IGNORE_VALUE,
        RC_RELEASE_VALUE,
        RC_RELEASE_VALUE,
    ]


def check_axis_override_channel_map() -> None:
    # ArduSub 4.1.2 initializes RC input as CH1=pitch, CH2=roll.
    assert AXIS_TO_CHANNEL == {
        "pitch": 0,
        "roll": 1,
        "heave": 2,
        "yaw": 3,
        "forward": 4,
        "lateral": 5,
    }


def main() -> int:
    check_gui_rc_input_span_contract()
    check_neutral_frame()
    check_sanitize_primary_rc()
    check_normalize_ardusub_rc_override()
    check_axis_override_channel_map()
    print("rc_frame_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

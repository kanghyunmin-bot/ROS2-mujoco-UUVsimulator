#!/usr/bin/env python3
"""Regression smoke for best-effort DVL message builders."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_dvl_messages import build_dvl_msg, build_dvldr_msg  # noqa: E402


class Header:
    stamp = None
    frame_id = ""


class DvlMsg:
    def __init__(self) -> None:
        self.header = Header()
        self.velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.altitude = 0.0
        self.time = 0.0
        self.fom = 0.0
        self.covariance = []
        self.beams = []
        self.velocity_valid = False
        self.status = -1
        self.time_of_validity = -1
        self.time_of_transmission = -1
        self.form = ""


class DvldrMsg:
    def __init__(self) -> None:
        self.header = Header()
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.yaw = 0.0
        self.roll = 1.0
        self.pitch = 1.0


def check_dvl_msg() -> None:
    msg = build_dvl_msg(DvlMsg, "stamp", np.array([1.0, -2.0, 3.0]), 4.5)
    assert msg.header.stamp == "stamp"
    assert msg.header.frame_id == "dvl"
    assert (msg.velocity.x, msg.velocity.y, msg.velocity.z) == (1.0, -2.0, 3.0)
    assert msg.altitude == 4.5
    assert msg.velocity_valid is True
    assert len(msg.covariance) == 9
    assert msg.time == 20.0
    assert msg.status == 0
    assert msg.form == "simulated_a50_velocity"
    assert len(msg.beams) == 4
    assert all(beam.valid for beam in msg.beams)
    assert all(beam.distance > msg.altitude for beam in msg.beams)


def check_installed_dvl_schema() -> None:
    try:
        from builtin_interfaces.msg import Time
        from dvl_msgs.msg import DVL
    except ImportError:
        print("ros2_dvl_installed_schema=NOT_RUN reason=dvl_msgs unavailable")
        return
    fields = DVL.get_fields_and_field_types()
    required = {
        "header", "time", "velocity", "fom", "covariance", "altitude", "beams",
        "velocity_valid", "status", "time_of_validity", "time_of_transmission", "form",
    }
    assert set(fields) == required, fields
    stamp = Time(sec=123, nanosec=456_789_000)
    msg = build_dvl_msg(DVL, stamp, np.array([1.0, -2.0, 3.0]), 4.5)
    assert msg.velocity_valid is True
    assert msg.header.frame_id == "dvl"
    assert len(msg.covariance) == 9
    assert msg.time_of_validity == 123_456_789
    assert len(msg.beams) == 4
    assert all(beam.valid for beam in msg.beams)


def check_dvldr_msg() -> None:
    msg = build_dvldr_msg(DvldrMsg, "stamp", np.array([1.0, 2.0, -0.5]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert msg.header.stamp == "stamp"
    assert msg.header.frame_id == "odom"
    assert (msg.position.x, msg.position.y, msg.position.z) == (1.0, 2.0, -0.5)
    assert msg.roll == 0.0
    assert msg.pitch == 0.0
    assert abs(msg.yaw) < 1e-9


def main() -> int:
    check_dvl_msg()
    check_installed_dvl_schema()
    check_dvldr_msg()
    print("ros2_dvl_messages=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

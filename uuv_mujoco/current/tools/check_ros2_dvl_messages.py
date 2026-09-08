#!/usr/bin/env python3
"""Validate the physical A50 source/installed schemas and message builders."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = ROOT.parents[1]
A50_MSG_ROOT = REPO_ROOT / "rospkg" / "src" / "auv_dvl_a50_msg" / "msg"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_dvl_messages import build_dvl_msg, build_dvldr_msg  # noqa: E402
from bridge.dvl_a50_sensor_model import A50SensorConfig, A50SensorModel  # noqa: E402


SOURCE_DVL_SCHEMA = (
    ("header", "std_msgs/Header"),
    ("time", "float64"),
    ("velocity", "geometry_msgs/Vector3"),
    ("fom", "float64"),
    ("covariance", "float64[]"),
    ("altitude", "float64"),
    ("beams", "DVLBeam[]"),
    ("velocity_valid", "bool"),
    ("status", "int64"),
    ("time_of_validity", "int64"),
    ("time_of_transmission", "int64"),
    ("form", "string"),
)
SOURCE_DVLDR_SCHEMA = (
    ("header", "std_msgs/Header"),
    ("time", "float64"),
    ("position", "geometry_msgs/Vector3"),
    ("pos_std", "float64"),
    ("roll", "float64"),
    ("pitch", "float64"),
    ("yaw", "float64"),
    ("type", "string"),
    ("status", "int64"),
    ("format", "string"),
)
INSTALLED_DVL_SCHEMA = (
    ("header", "std_msgs/Header"),
    ("time", "double"),
    ("velocity", "geometry_msgs/Vector3"),
    ("fom", "double"),
    ("covariance", "sequence<double>"),
    ("altitude", "double"),
    ("beams", "sequence<auv_dvl_a50_msg/DVLBeam>"),
    ("velocity_valid", "boolean"),
    ("status", "int64"),
    ("time_of_validity", "int64"),
    ("time_of_transmission", "int64"),
    ("form", "string"),
)
INSTALLED_DVLDR_SCHEMA = (
    ("header", "std_msgs/Header"),
    ("time", "double"),
    ("position", "geometry_msgs/Vector3"),
    ("pos_std", "double"),
    ("roll", "double"),
    ("pitch", "double"),
    ("yaw", "double"),
    ("type", "string"),
    ("status", "int64"),
    ("format", "string"),
)


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
        self.time = 0.0
        self.pos_std = 0.0
        self.type = ""
        self.status = -1
        self.format = ""


class Stamp:
    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.sec = sec
        self.nanosec = nanosec


def check_dvl_msg() -> None:
    msg = build_dvl_msg(DvlMsg, "stamp", np.array([1.0, -2.0, 3.0]), 4.5)
    assert msg.header.stamp == "stamp"
    assert msg.header.frame_id == "dvl_link"
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


def _parse_source_schema(path: Path) -> tuple[tuple[str, str], ...]:
    fields: list[tuple[str, str]] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", maxsplit=1)[0].strip()
        if not line:
            continue
        field_type, field_name = line.split()
        fields.append((field_name, field_type))
    return tuple(fields)


def check_source_schemas() -> None:
    assert _parse_source_schema(A50_MSG_ROOT / "DVL.msg") == SOURCE_DVL_SCHEMA
    assert _parse_source_schema(A50_MSG_ROOT / "DVLDR.msg") == SOURCE_DVLDR_SCHEMA
    print("ros2_dvl_source_schema=PASS package=auv_dvl_a50_msg")


def check_installed_dvl_schemas() -> None:
    try:
        from builtin_interfaces.msg import Time
        from auv_dvl_a50_msg.msg import DVL, DVLDR
    except ModuleNotFoundError as exc:
        if exc.name not in {"builtin_interfaces", "auv_dvl_a50_msg"}:
            raise
        print("ros2_dvl_installed_schema=NOT_RUN reason=auv_dvl_a50_msg unavailable")
        return
    assert tuple(DVL.get_fields_and_field_types().items()) == INSTALLED_DVL_SCHEMA
    assert tuple(DVLDR.get_fields_and_field_types().items()) == INSTALLED_DVLDR_SCHEMA
    stamp = Time(sec=123, nanosec=456_789_000)
    msg = build_dvl_msg(DVL, stamp, np.array([1.0, -2.0, 3.0]), 4.5)
    assert msg.velocity_valid is True
    assert msg.header.frame_id == "dvl_link"
    assert len(msg.covariance) == 9
    assert msg.time_of_validity == 123_456_789
    assert len(msg.beams) == 4
    assert all(beam.valid for beam in msg.beams)
    dr_msg = build_dvldr_msg(
        DVLDR,
        stamp,
        np.array([1.0, 2.0, -0.5]),
        np.array([1.0, 0.0, 0.0, 0.0]),
        position_std_m=0.12,
        report_time_s=123.5,
    )
    assert dr_msg.header.frame_id == "dvl_link"
    assert dr_msg.format == "json_v3"
    print("ros2_dvl_installed_schema=PASS package=auv_dvl_a50_msg types=DVL,DVLDR")


def check_dvldr_msg() -> None:
    msg = build_dvldr_msg(
        DvldrMsg,
        "stamp",
        np.array([1.0, 2.0, -0.5]),
        np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]),
        position_std_m=0.12,
        report_time_s=123.5,
    )
    assert msg.header.stamp == "stamp"
    assert msg.header.frame_id == "dvl_link"
    assert (msg.position.x, msg.position.y, msg.position.z) == (1.0, 2.0, -0.5)
    assert msg.roll == 0.0
    assert msg.pitch == 0.0
    assert abs(msg.yaw - 90.0) < 1e-9
    assert msg.time == 123.5
    assert msg.pos_std == 0.12
    assert msg.type == "position_local"
    assert msg.status == 0
    assert msg.format == "json_v3"


def check_a50_sensor_sample_mapping() -> None:
    model = A50SensorModel(
        A50SensorConfig(
            seed=9,
            white_noise_std_mps=0.0,
            velocity_noise_per_meter_mps=0.0,
            range_noise_std_m=0.0,
            range_noise_fraction=0.0,
            transmission_delay_us=80_000,
        )
    )
    sample = model.sample(
        (0.4, -0.2, 0.05),
        0.5,
        time_of_validity_us=1_000_000,
        forced_dropout_beams={0},
    )
    msg = build_dvl_msg(
        DvlMsg,
        Stamp(sec=1),
        None,
        None,
        sample_period_s=0.1,
        sensor_sample=sample,
    )
    assert msg.header.frame_id == "dvl_link"
    assert msg.form == "json_v3.3"
    assert msg.time == 100.0
    assert msg.time_of_validity == 1_000_000
    assert msg.time_of_transmission == 1_080_000
    assert msg.velocity_valid is True
    assert len(msg.beams) == 4
    assert msg.beams[0].valid is False
    assert msg.beams[0].distance == -1.0
    assert all(-120.0 <= beam.rssi <= -10.0 for beam in msg.beams)
    assert all(-120.0 <= beam.nsd <= -60.0 for beam in msg.beams)


def main() -> int:
    check_source_schemas()
    check_dvl_msg()
    check_installed_dvl_schemas()
    check_dvldr_msg()
    check_a50_sensor_sample_mapping()
    print("ros2_dvl_messages=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

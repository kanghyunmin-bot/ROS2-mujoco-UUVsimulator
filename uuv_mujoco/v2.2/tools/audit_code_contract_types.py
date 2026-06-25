"""Shared data types for source-level contract audits."""

from __future__ import annotations

from dataclasses import dataclass


OFFICIAL_REFS = {
    "ardupilot_json_sitl": "https://ardupilot.org/dev/docs/sitl-with-JSON.html",
    "mavlink_servo_output_raw": "https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW",
    "mavlink_rc_channels_override": "https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE",
    "mujoco_fluid": "https://mujoco.readthedocs.io/en/3.3.3/computation/fluid.html",
    "bar30_pressure_sensor": "https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/",
}


@dataclass
class Evidence:
    path: str
    line: int | None
    snippet: str


@dataclass
class Check:
    check_id: str
    status: str
    title: str
    conclusion: str
    evidence: list[Evidence]
    official_refs: list[str]

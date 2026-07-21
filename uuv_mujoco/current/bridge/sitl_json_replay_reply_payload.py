"""Payload helpers for immediate sensor-replay JSON replies."""

from __future__ import annotations

from typing import Any

from bridge.sitl_json_sensor_replay_apply import vertical_estimate_from_replay_frame
from bridge.sitl_math import quat_to_rpy


def immediate_replay_state(replay_frame: Any):
    vertical_est = vertical_estimate_from_replay_frame(replay_frame)
    roll, pitch, yaw = quat_to_rpy(replay_frame.quat_ned_frd)
    return vertical_est, roll, pitch, yaw


def send_immediate_replay_external_nav(
    self,
    sensor_time_s: float,
    replay_frame: Any,
    vertical_est: Any,
    roll: float,
    pitch: float,
    yaw: float,
) -> None:
    self._send_external_nav(float(sensor_time_s), vertical_est, replay_frame.quat_ned_frd, roll, pitch, yaw)
    self._enforce_extnav_contract()


def build_immediate_replay_payload(
    self,
    sensor_time_s: float,
    replay_frame: Any,
    vertical_est: Any,
    roll: float,
    pitch: float,
    yaw: float,
) -> dict[str, object]:
    return self._payload_from_state(
        float(sensor_time_s),
        replay_frame.gyro_frd,
        replay_frame.accel_frd,
        vertical_est,
        replay_frame.quat_ned_frd,
        roll,
        pitch,
        yaw,
        None,
    )


__all__ = [
    "build_immediate_replay_payload",
    "immediate_replay_state",
    "send_immediate_replay_external_nav",
]

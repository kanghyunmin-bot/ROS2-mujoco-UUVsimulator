#!/usr/bin/env python3
"""Smoke-check immediate sensor replay replies without SITL or ROS."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.sitl_json_replay_reply import _send_immediate_sensor_replay_reply  # noqa: E402


@dataclass
class ReplayFrame:
    t_s: float = 12.0
    depth_m: float = 0.25
    pressure_pa: float = 101350.0
    pos_ned: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.25], dtype=float))
    vel_ned: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    alt_m: float = -0.25
    extnav_pos_ned: np.ndarray | None = None
    quat_ned_frd: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=float))
    gyro_frd: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    accel_frd: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 9.81], dtype=float))


class FakeTransport:
    def __init__(self, *, sensor_time: float | None = 5.0) -> None:
        self._sensor_replay_immediate_reply = True
        self._sensor_replay_frames = [ReplayFrame()]
        self._sensor_replay_clock = "servo_frame"
        self.sitl_sock = object()
        self._sensor_replay_immediate_last_frame_count = -1
        self._sensor_replay_current_payload_t_s = sensor_time
        self._sensor_replay_current_clock_t = None
        self._sensor_replay_live_rangefinder = False
        self._sensor_replay_immediate_send_counter = 0
        self._sitl_cmd_debug = False
        self._sensor_replay_immediate_last_log_wall = 0.0
        self.external_nav_calls = 0
        self.json_payloads: list[dict[str, object]] = []

    def _sensor_replay_frame_at(self, _t: float):
        return self._sensor_replay_frames[0]

    def _send_external_nav(self, *args) -> None:
        self.external_nav_calls += 1

    def _enforce_extnav_contract(self) -> None:
        return None

    def _payload_from_state(self, sensor_time_s, gyro, acc, vertical_est, quat, roll, pitch, yaw, rng):
        return {
            "timestamp": float(sensor_time_s),
            "gyro": list(gyro),
            "acc": list(acc),
            "depth_m": vertical_est.depth_m,
            "rng": rng,
        }

    def _send_sitl_json_payload(self, payload, *, now_wall, vertical_est, pressure_pa) -> None:
        self.json_payloads.append(payload)


def test_one_reply_per_servo_frame() -> None:
    transport = FakeTransport(sensor_time=5.0)
    _send_immediate_sensor_replay_reply(transport, 10.0, 7)
    _send_immediate_sensor_replay_reply(transport, 10.1, 7)
    assert transport.external_nav_calls == 1
    assert len(transport.json_payloads) == 1
    assert transport.json_payloads[0]["timestamp"] == 5.0
    assert transport._sensor_replay_immediate_send_counter == 1
    assert transport._sensor_replay_immediate_last_frame_count == 7


def test_missing_sensor_time_blocks_reply() -> None:
    transport = FakeTransport(sensor_time=None)
    _send_immediate_sensor_replay_reply(transport, 10.0, 8)
    assert transport.external_nav_calls == 0
    assert not transport.json_payloads
    assert transport._sensor_replay_immediate_last_frame_count == -1


def main() -> int:
    test_one_reply_per_servo_frame()
    test_missing_sensor_time_blocks_reply()
    print("immediate_sensor_replay_reply=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

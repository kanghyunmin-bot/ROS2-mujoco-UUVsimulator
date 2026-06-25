"""SITL JSON sensor send policy for live and replayed state."""

from __future__ import annotations

import time

import numpy as np

from bridge.sitl_json_sensor_external_nav import dispatch_json_sensor_external_nav
from bridge.sitl_json_sensor_replay_apply import (
    apply_sensor_replay_frame,
    publish_loop_blocked_by_immediate_replay,
    sensor_payload_time_s,
)
from bridge.sitl_math import quat_to_rpy
from bridge.sitl_types import VerticalEstimate


def send_state(
    self,
    sim_t: float,
    gyro: np.ndarray,
    acc: np.ndarray,
    vertical_est: VerticalEstimate,
    quat: np.ndarray,
    rangefinder_distance_m: float | None = None,
    pressure_pa: float | None = None,
) -> None:
    """Send JSON packet to ArduPilot SITL."""
    if not self.sitl_sock:
        return
    if publish_loop_blocked_by_immediate_replay(self):
        # In controller-parity full-runtime mode the JSON sensor stream is
        # driven directly by ArduSub JSON servo frames, matching the
        # isolated replay harness. Avoid sending duplicate publish-loop
        # packets with a different phase.
        return

    now_wall = time.monotonic()
    sitl_t = float(max(0.0, sim_t))
    replay_frame = self._sensor_replay_frame_at(sitl_t)
    if replay_frame is not None:
        gyro, acc, vertical_est, quat, pressure_pa, rangefinder_distance_m = apply_sensor_replay_frame(
            self,
            replay_frame,
            now_wall,
            rangefinder_distance_m,
        )

    sensor_time_s = sensor_payload_time_s(self, sitl_t, replay_frame)
    roll, pitch, yaw = quat_to_rpy(quat)
    dispatch_json_sensor_external_nav(self, sitl_t, vertical_est, quat, roll, pitch, yaw)
    payload = self._payload_from_state(
        sensor_time_s,
        gyro,
        acc,
        vertical_est,
        quat,
        roll,
        pitch,
        yaw,
        rangefinder_distance_m,
    )
    self._send_sitl_json_payload(
        payload,
        now_wall=now_wall,
        vertical_est=vertical_est,
        pressure_pa=pressure_pa,
    )


__all__ = ["send_state"]

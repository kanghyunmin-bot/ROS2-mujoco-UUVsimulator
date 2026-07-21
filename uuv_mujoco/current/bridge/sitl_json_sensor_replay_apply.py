"""Sensor-replay substitution helpers for SITL JSON sensor packets."""

from __future__ import annotations

from typing import Any

import numpy as np

from bridge.sitl_types import VerticalEstimate


def publish_loop_blocked_by_immediate_replay(self) -> bool:
    return bool(
        self._sensor_replay_immediate_reply
        and self._sensor_replay_frames
        and self._sensor_replay_clock == "servo_frame"
    )


def vertical_estimate_from_replay_frame(replay_frame: Any) -> VerticalEstimate:
    return VerticalEstimate(
        depth_m=float(replay_frame.depth_m),
        pressure_pa=replay_frame.pressure_pa,
        pos_ned=replay_frame.pos_ned,
        vel_ned=replay_frame.vel_ned,
        alt_m=float(replay_frame.alt_m),
        extnav_pos_ned=replay_frame.extnav_pos_ned,
    )


def log_sensor_replay_sample(self, replay_frame: Any, now_wall: float) -> None:
    if not (self._sitl_cmd_debug and now_wall - self._sensor_replay_last_log_wall >= 2.0):
        return
    self._sensor_replay_last_log_wall = now_wall
    print(
        "[sitl_transport] controller-parity sensor replay sample "
        f"t_replay={replay_frame.t_s:.3f} depth={replay_frame.depth_m:.3f} "
        f"vel_d={replay_frame.vel_ned[2]:+.3f}",
        flush=True,
    )


def apply_sensor_replay_frame(
    self,
    replay_frame: Any,
    now_wall: float,
    rangefinder_distance_m: float | None,
) -> tuple[np.ndarray, np.ndarray, VerticalEstimate, np.ndarray, float | None, float | None]:
    vertical_est = vertical_estimate_from_replay_frame(replay_frame)
    pressure_pa = replay_frame.pressure_pa
    if not self._sensor_replay_live_rangefinder:
        # Controller-parity replay must not mix live MuJoCo range data into a
        # recorded sensor stream. The isolated harness omits rng_1 unless it is
        # explicitly requested.
        rangefinder_distance_m = None
    log_sensor_replay_sample(self, replay_frame, now_wall)
    return (
        replay_frame.gyro_frd,
        replay_frame.accel_frd,
        vertical_est,
        replay_frame.quat_ned_frd,
        pressure_pa,
        rangefinder_distance_m,
    )


def sensor_payload_time_s(self, sitl_t: float, replay_frame: Any | None) -> float:
    replay_payload_timestamp_s = self._sensor_replay_payload_timestamp_for_sim_t(sitl_t)
    if replay_frame is not None and replay_payload_timestamp_s is not None:
        # Match sensor_replay_sitl_json.py: pre-roll sends bootstrap timestamps
        # 0..pre_roll_s, then replay sends json_time_offset_s + replay_s. The
        # full MuJoCo runtime must not mix replayed sensor content with live
        # MuJoCo sim_t, or ArduSub's EKF/controller state diverges during mode
        # re-entry transients.
        self._sensor_replay_current_payload_t_s = float(replay_payload_timestamp_s)
        return float(replay_payload_timestamp_s)
    return float(sitl_t)


__all__ = [
    "apply_sensor_replay_frame",
    "log_sensor_replay_sample",
    "publish_loop_blocked_by_immediate_replay",
    "sensor_payload_time_s",
    "vertical_estimate_from_replay_frame",
]

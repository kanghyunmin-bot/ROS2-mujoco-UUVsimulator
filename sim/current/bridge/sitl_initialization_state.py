"""Mutable sensor replay state reset helpers."""

from __future__ import annotations


def reset_sensor_replay_runtime_state(transport: object) -> None:
    transport._sensor_replay_rc_seen = False
    transport._sensor_replay_first_rc_sim_t: float | None = None
    transport._sensor_replay_start_sim_t: float | None = None
    transport._sensor_replay_first_rc_clock_t: float | None = None
    transport._sensor_replay_start_clock_t: float | None = None
    transport._sensor_replay_current_clock_t: float | None = None
    transport._sensor_replay_current_t_s: float | None = None
    transport._sensor_replay_current_payload_t_s: float | None = None
    transport._sensor_replay_current_frame = None
    transport._sensor_replay_last_clock_t_s: float | None = None
    transport._sensor_replay_ready = False
    transport._sitl_json_first_frame_count: int | None = None
    transport._sitl_json_latest_frame_count: int | None = None
    transport._sitl_json_latest_frame_rate_hz: int | None = None
    transport._sensor_replay_start_wait_log_wall = -1.0
    transport._sensor_replay_index = 0
    transport._sensor_replay_last_log_wall = -1.0


__all__ = ["reset_sensor_replay_runtime_state"]

"""Payload timestamp policy for controller-parity sensor replay."""

from __future__ import annotations


def sensor_replay_payload_timestamp_for_sim_t(transport, sim_t: float) -> float | None:
    if not transport._sensor_replay_frames or not transport._sensor_replay_ready:
        return None
    if transport._sensor_replay_start_on_rc:
        if transport._sensor_replay_start_sim_t is None:
            return None
        replay_elapsed_s = max(0.0, float(sim_t) - float(transport._sensor_replay_start_sim_t))
        return (
            float(transport._sensor_replay_start_delay_s)
            + float(transport._sensor_replay_time_offset_s)
            + float(replay_elapsed_s)
        )
    if transport._sensor_replay_current_t_s is None:
        return None
    return float(transport._sensor_replay_current_t_s)


__all__ = ["sensor_replay_payload_timestamp_for_sim_t"]

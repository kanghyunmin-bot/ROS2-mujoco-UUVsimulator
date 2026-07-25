"""Servo-frame clock policy for controller-parity sensor replay."""

from __future__ import annotations


def sensor_replay_servo_frame_clock_time_s(transport) -> float | None:
    frame_count = transport._sitl_json_latest_frame_count
    frame_rate_hz = transport._sitl_json_latest_frame_rate_hz
    if frame_count is None or frame_rate_hz is None or frame_rate_hz <= 0:
        return None
    if transport._sitl_json_first_frame_count is None:
        transport._sitl_json_first_frame_count = int(frame_count)
    clock_t = max(
        0.0,
        (int(frame_count) - int(transport._sitl_json_first_frame_count))
        / max(float(frame_rate_hz), 1.0),
    )
    # UDP can expose a just-received frame_count before the cached latest value
    # settles, especially around the first RC edge. Match the isolated replay
    # harness by enforcing a monotonic servo-frame clock.
    if transport._sensor_replay_last_clock_t_s is not None:
        clock_t = max(float(clock_t), float(transport._sensor_replay_last_clock_t_s))
    transport._sensor_replay_last_clock_t_s = float(clock_t)
    return float(clock_t)


__all__ = ["sensor_replay_servo_frame_clock_time_s"]

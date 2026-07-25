"""Sensor replay status payload builder."""

from __future__ import annotations

from bridge.sitl_replay import SensorReplayFrame


def build_sensor_replay_status(
    *,
    frame: SensorReplayFrame | None,
    frame_rpy: tuple[float | None, float | None, float | None],
    active: bool,
    clock: str,
    ready: bool,
    current_clock_t_s: float | None,
    current_t_s: float | None,
    payload_timestamp_s: float | None,
    real_start_s: float,
    start_delay_s: float,
    first_rc_clock_t_s: float | None,
    start_clock_t_s: float | None,
    json_frame_count: int | None,
    json_frame_rate_hz: int | None,
) -> dict[str, object]:
    frame_roll, frame_pitch, frame_yaw = frame_rpy
    return {
        "active": bool(active),
        "clock": str(clock),
        "ready": bool(ready),
        "current_clock_t_s": current_clock_t_s,
        "current_t_s": current_t_s,
        "current_real_t_s": (
            float(current_t_s) + float(real_start_s) if current_t_s is not None else None
        ),
        "payload_timestamp_s": payload_timestamp_s,
        "real_start_s": float(real_start_s),
        "start_delay_s": float(start_delay_s),
        "first_rc_clock_t_s": first_rc_clock_t_s,
        "start_clock_t_s": start_clock_t_s,
        "json_frame_count": json_frame_count,
        "json_frame_rate_hz": json_frame_rate_hz,
        "frame_depth_m": float(frame.depth_m) if frame is not None else None,
        "frame_vel_d_mps": float(frame.vel_ned[2]) if frame is not None else None,
        "frame_pressure_pa": float(frame.pressure_pa) if frame is not None else None,
        "frame_roll_rad": frame_roll,
        "frame_pitch_rad": frame_pitch,
        "frame_yaw_rad": frame_yaw,
        "frame_gyro_x_radps": float(frame.gyro_frd[0]) if frame is not None else None,
        "frame_gyro_y_radps": float(frame.gyro_frd[1]) if frame is not None else None,
        "frame_gyro_z_radps": float(frame.gyro_frd[2]) if frame is not None else None,
    }


__all__ = ["build_sensor_replay_status"]

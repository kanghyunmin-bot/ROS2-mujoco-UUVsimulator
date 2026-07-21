"""Gate and timestamp helpers for immediate sensor-replay JSON replies."""

from __future__ import annotations


def immediate_replay_reply_due(self, frame_count: int) -> bool:
    if (
        not self._sensor_replay_immediate_reply
        or not self._sensor_replay_frames
        or self._sensor_replay_clock != "servo_frame"
        or not self.sitl_sock
    ):
        return False
    return self._sensor_replay_immediate_last_frame_count != int(frame_count)


def immediate_replay_sensor_time_s(self) -> float | None:
    sensor_time_s = self._sensor_replay_current_payload_t_s
    if sensor_time_s is None:
        sensor_time_s = self._sensor_replay_current_clock_t
    return None if sensor_time_s is None else float(sensor_time_s)


__all__ = ["immediate_replay_reply_due", "immediate_replay_sensor_time_s"]

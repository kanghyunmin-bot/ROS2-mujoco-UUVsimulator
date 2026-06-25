"""Immediate sensor-replay replies synchronized to JSON servo frames."""

from __future__ import annotations

from bridge.sitl_json_replay_reply_gate import immediate_replay_reply_due, immediate_replay_sensor_time_s
from bridge.sitl_json_replay_reply_log import mark_immediate_replay_reply_sent
from bridge.sitl_json_replay_reply_payload import (
    build_immediate_replay_payload,
    immediate_replay_state,
    send_immediate_replay_external_nav,
)


def _send_immediate_sensor_replay_reply(self, now_wall: float, frame_count: int) -> None:
    if not immediate_replay_reply_due(self, int(frame_count)):
        return

    replay_frame = self._sensor_replay_frame_at(0.0)
    sensor_time_s = immediate_replay_sensor_time_s(self)
    if replay_frame is None or sensor_time_s is None:
        return

    vertical_est, roll, pitch, yaw = immediate_replay_state(replay_frame)
    send_immediate_replay_external_nav(self, sensor_time_s, replay_frame, vertical_est, roll, pitch, yaw)
    payload = build_immediate_replay_payload(self, sensor_time_s, replay_frame, vertical_est, roll, pitch, yaw)
    self._send_sitl_json_payload(
        payload,
        now_wall=now_wall,
        vertical_est=vertical_est,
        pressure_pa=replay_frame.pressure_pa,
    )
    mark_immediate_replay_reply_sent(
        self,
        now_wall=now_wall,
        frame_count=int(frame_count),
        sensor_time_s=sensor_time_s,
        replay_frame=replay_frame,
    )


__all__ = ["_send_immediate_sensor_replay_reply"]

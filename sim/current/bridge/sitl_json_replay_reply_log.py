"""Counter and debug-log helpers for immediate sensor-replay replies."""

from __future__ import annotations

from typing import Any


def mark_immediate_replay_reply_sent(
    self,
    *,
    now_wall: float,
    frame_count: int,
    sensor_time_s: float,
    replay_frame: Any,
) -> None:
    self._sensor_replay_immediate_last_frame_count = int(frame_count)
    self._sensor_replay_immediate_send_counter += 1
    log_immediate_replay_reply(self, now_wall, frame_count, sensor_time_s, replay_frame)


def log_immediate_replay_reply(
    self,
    now_wall: float,
    frame_count: int,
    sensor_time_s: float,
    replay_frame: Any,
) -> None:
    if not (self._sitl_cmd_debug and now_wall - self._sensor_replay_immediate_last_log_wall >= 2.0):
        return
    self._sensor_replay_immediate_last_log_wall = now_wall
    print(
        "[sitl_transport] controller-parity immediate sensor reply "
        f"frame={int(frame_count)} t={float(sensor_time_s):.3f} "
        f"replay_t={float(replay_frame.t_s):.3f} "
        f"sent={self._sensor_replay_immediate_send_counter}",
        flush=True,
    )


__all__ = ["log_immediate_replay_reply", "mark_immediate_replay_reply_sent"]

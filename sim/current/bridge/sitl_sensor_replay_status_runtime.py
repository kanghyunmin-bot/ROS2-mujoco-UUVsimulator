"""Status reporting for controller-parity sensor replay."""

from __future__ import annotations

from bridge.sitl_math import quat_to_rpy
from bridge.sitl_status import build_sensor_replay_status


def sensor_replay_status(self) -> dict[str, object]:
    frame = self._sensor_replay_current_frame
    frame_rpy: tuple[float | None, float | None, float | None] = (None, None, None)
    if frame is not None:
        try:
            frame_rpy = quat_to_rpy(frame.quat_ned_frd)
        except Exception:
            frame_rpy = (None, None, None)
    status = build_sensor_replay_status(
        frame=frame,
        frame_rpy=frame_rpy,
        active=bool(self._sensor_replay_frames),
        clock=self._sensor_replay_clock,
        ready=bool(self._sensor_replay_ready),
        current_clock_t_s=self._sensor_replay_current_clock_t,
        current_t_s=self._sensor_replay_current_t_s,
        payload_timestamp_s=self._sensor_replay_current_payload_t_s,
        real_start_s=float(self._sensor_replay_real_start_s),
        start_delay_s=float(self._sensor_replay_start_delay_s),
        first_rc_clock_t_s=self._sensor_replay_first_rc_clock_t,
        start_clock_t_s=self._sensor_replay_start_clock_t,
        json_frame_count=self._sitl_json_latest_frame_count,
        json_frame_rate_hz=self._sitl_json_latest_frame_rate_hz,
    )
    live_payload = getattr(self, "_sitl_last_json_payload_status", None)
    if isinstance(live_payload, dict):
        status.update(live_payload)
    return status


__all__ = ["sensor_replay_status"]

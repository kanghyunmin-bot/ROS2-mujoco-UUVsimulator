"""Frame-selection policy for controller-parity sensor replay."""

from __future__ import annotations

from bridge.sitl_replay import SensorReplayFrame, interpolate_sensor_replay_frame
from bridge.sitl_sensor_replay_hold_policy import _hold_waiting_for_servo_clock
from bridge.sitl_sensor_replay_start_policy import _replay_time_after_start_policy


def _sensor_replay_frame_at(self, sim_t: float) -> SensorReplayFrame | None:
    frames = self._sensor_replay_frames
    if not frames:
        return None
    clock_t = self._sensor_replay_clock_time_s(sim_t)
    self._sensor_replay_current_clock_t = float(clock_t) if clock_t is not None else None
    if clock_t is None:
        return _hold_waiting_for_servo_clock(self, frames)

    t_s, held_frame = _replay_time_after_start_policy(self, frames, sim_t, float(clock_t))
    if held_frame is not None:
        return held_frame

    self._sensor_replay_current_t_s = float(t_s)
    frame, frame_index = interpolate_sensor_replay_frame(
        frames,
        t_s=t_s,
        start_index=self._sensor_replay_index,
        surface_pressure_pa=self._bar30_surface_pressure_pa,
        water_density=self._bar30_water_density,
        gravity=self._bar30_gravity,
    )
    if frame_index is not None:
        self._sensor_replay_index = int(frame_index)
    return self._remember_sensor_replay_frame(frame)


__all__ = ["_sensor_replay_frame_at"]

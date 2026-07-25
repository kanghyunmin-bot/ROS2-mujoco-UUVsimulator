"""Hold-state helpers for controller-parity sensor replay."""

from __future__ import annotations

import time

from bridge.sitl_replay import SensorReplayFrame


def _log_wait_message(self, message: str) -> None:
    now_wall = time.monotonic()
    if self._sitl_cmd_debug and now_wall - self._sensor_replay_start_wait_log_wall >= 3.0:
        self._sensor_replay_start_wait_log_wall = now_wall
        print(message, flush=True)


def _hold_first_replay_frame(self, frames: list[SensorReplayFrame], *, ready: bool = False) -> SensorReplayFrame:
    self._sensor_replay_current_t_s = float(frames[0].t_s)
    self._sensor_replay_current_payload_t_s = None
    self._sensor_replay_ready = bool(ready)
    return self._remember_sensor_replay_frame(frames[0])


def _hold_waiting_for_servo_clock(self, frames: list[SensorReplayFrame]) -> SensorReplayFrame:
    _log_wait_message(
        self,
        "[sitl_transport] controller-parity sensor replay waiting for "
        "JSON servo frame_count before starting servo_frame clock",
    )
    return _hold_first_replay_frame(self, frames)


def _hold_until_rc_input(self, frames: list[SensorReplayFrame]) -> SensorReplayFrame:
    _log_wait_message(
        self,
        "[sitl_transport] controller-parity sensor replay holding first frame "
        "until real RC override is forwarded",
    )
    # Before the real RC edge, keep using the live SITL time for
    # JSON/ExternalNav timestamps so MAVROS/EKF bootstrap can connect. The
    # replay pre-roll clock starts only after RC is forwarded, matching the
    # isolated parity harness.
    return _hold_first_replay_frame(self, frames)


def _hold_rc_preroll(self, frames: list[SensorReplayFrame], clock_t: float) -> SensorReplayFrame:
    _log_wait_message(
        self,
        "[sitl_transport] controller-parity sensor replay in RC pre-roll "
        f"elapsed={float(clock_t) - float(self._sensor_replay_first_rc_clock_t):.3f}s/"
        f"{self._sensor_replay_start_delay_s:.3f}s",
    )
    self._sensor_replay_current_t_s = float(frames[0].t_s)
    self._sensor_replay_current_payload_t_s = max(
        0.0,
        float(clock_t) - float(self._sensor_replay_first_rc_clock_t),
    )
    self._sensor_replay_ready = False
    return self._remember_sensor_replay_frame(frames[0])


__all__ = [
    "_hold_first_replay_frame",
    "_hold_waiting_for_servo_clock",
    "_hold_until_rc_input",
    "_hold_rc_preroll",
]

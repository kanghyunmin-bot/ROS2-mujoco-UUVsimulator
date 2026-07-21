"""Start-time policy for controller-parity sensor replay."""

from __future__ import annotations

from bridge.sitl_replay import SensorReplayFrame
from bridge.sitl_sensor_replay_hold_policy import _hold_rc_preroll, _hold_until_rc_input


def _mark_first_rc_clock(self, sim_t: float, clock_t: float) -> None:
    if self._sensor_replay_first_rc_clock_t is not None:
        return
    self._sensor_replay_first_rc_clock_t = float(clock_t)
    self._sensor_replay_first_rc_sim_t = float(sim_t)
    print(
        "[sitl_transport] controller-parity sensor replay saw first RC "
        f"at sim_t={float(sim_t):.3f}s clock_t={float(clock_t):.3f}s; "
        "holding bootstrap frame for "
        f"{self._sensor_replay_start_delay_s:.3f}s",
        flush=True,
    )


def _start_replay_after_preroll(self, sim_t: float, start_clock_t: float) -> None:
    if self._sensor_replay_start_clock_t is not None:
        return
    self._sensor_replay_start_clock_t = start_clock_t
    self._sensor_replay_start_sim_t = float(sim_t)
    self._sensor_replay_index = 0
    self._sensor_replay_ready = True
    print(
        "[sitl_transport] controller-parity sensor replay clock started "
        f"at clock_t={start_clock_t:.3f}s "
        f"(sim_t={float(sim_t):.3f}s) after RC pre-roll",
        flush=True,
    )


def _replay_time_after_start_policy(
    self,
    frames: list[SensorReplayFrame],
    sim_t: float,
    clock_t: float,
) -> tuple[float, SensorReplayFrame | None]:
    if not self._sensor_replay_start_on_rc:
        self._sensor_replay_ready = True
        t_s = float(clock_t) + float(self._sensor_replay_time_offset_s)
        self._sensor_replay_current_payload_t_s = float(t_s)
        return float(t_s), None

    if not self._sensor_replay_rc_seen:
        return 0.0, _hold_until_rc_input(self, frames)
    _mark_first_rc_clock(self, sim_t, clock_t)
    start_clock_t = float(self._sensor_replay_first_rc_clock_t) + float(self._sensor_replay_start_delay_s)
    if float(clock_t) < start_clock_t:
        return 0.0, _hold_rc_preroll(self, frames, clock_t)
    _start_replay_after_preroll(self, sim_t, start_clock_t)
    replay_elapsed_s = max(0.0, float(clock_t) - float(self._sensor_replay_start_clock_t))
    t_s = replay_elapsed_s + float(self._sensor_replay_time_offset_s)
    self._sensor_replay_current_payload_t_s = float(self._sensor_replay_start_delay_s) + float(t_s)
    return float(t_s), None


__all__ = [
    "_mark_first_rc_clock",
    "_start_replay_after_preroll",
    "_replay_time_after_start_policy",
]

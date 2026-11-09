"""Check that selected sensor frames and JSON timestamps share one clock."""

from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bridge.sitl_replay_types import SensorReplayFrame  # noqa: E402
from bridge.sitl_sensor_replay_frame_policy import _sensor_replay_frame_at  # noqa: E402
from bridge.sitl_sensor_replay_payload_time import sensor_replay_payload_timestamp_for_sim_t  # noqa: E402


def transport(clock: str, offset: float) -> SimpleNamespace:
    def frame(t: float) -> SensorReplayFrame:
        return SensorReplayFrame(
            t, np.zeros(3), np.array([0.0, 0.0, -9.80665]),
            np.array([1.0, 0.0, 0.0, 0.0]), t, None,
            np.array([0.0, 0.0, t]), np.zeros(3), -t, None,
        )

    state = SimpleNamespace(
        _sensor_replay_frames=[frame(0.0), frame(20.0)],
        _sensor_replay_clock=clock,
        _sensor_replay_start_on_rc=True,
        _sensor_replay_rc_seen=True,
        _sensor_replay_first_rc_clock_t=8.0,
        _sensor_replay_start_delay_s=2.0,
        _sensor_replay_start_clock_t=10.0,
        _sensor_replay_start_sim_t=100.0,
        _sensor_replay_time_offset_s=offset,
        _sensor_replay_current_payload_t_s=None,
        _sensor_replay_current_t_s=None,
        _sensor_replay_index=0,
        _sensor_replay_ready=True,
        _bar30_surface_pressure_pa=101325.0,
        _bar30_water_density=1000.0,
        _bar30_gravity=9.80665,
    )
    state._remember_sensor_replay_frame = lambda value: value
    return state


@pytest.mark.parametrize("offset", [0.0, 5.0])
def test_scheduled_servo_clock_does_not_relabel_sensor_history(offset: float) -> None:
    state = transport("servo_frame", offset)
    timestamps = []
    for servo_t, sim_t in [(10.0, 100.0), (10.25, 101.0), (10.5, 103.0)]:
        state._sensor_replay_clock_time_s = lambda _sim_t, value=servo_t: value
        selected = _sensor_replay_frame_at(state, sim_t)
        stamp = sensor_replay_payload_timestamp_for_sim_t(state, sim_t)
        assert stamp == pytest.approx(2.0 + selected.t_s)
        timestamps.append(stamp)
    assert np.diff(timestamps).tolist() == pytest.approx([0.25, 0.25])


def test_sim_time_mode_retains_existing_contract() -> None:
    state = transport("sim_time", 5.0)
    state._sensor_replay_start_clock_t = 100.0
    state._sensor_replay_clock_time_s = float
    selected = _sensor_replay_frame_at(state, 101.0)
    assert selected.t_s == 6.0
    assert sensor_replay_payload_timestamp_for_sim_t(state, 101.0) == 8.0


@pytest.mark.parametrize("ready,frames", [(False, True), (True, False)])
def test_unready_replay_has_no_payload_timestamp(ready: bool, frames: bool) -> None:
    state = transport("servo_frame", 0.0)
    state._sensor_replay_ready = ready
    if not frames:
        state._sensor_replay_frames = []
    assert sensor_replay_payload_timestamp_for_sim_t(state, 101.0) is None

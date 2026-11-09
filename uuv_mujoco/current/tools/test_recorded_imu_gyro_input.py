"""Verify gyro-stage inversion and causal use of recorded AHRS observations."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "debug/controller_parity_412/recorded_imu_gyro_input.py"
SPEC = importlib.util.spec_from_file_location("recorded_gyro", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_recovers_known_ins_input_and_preserves_original_arrays() -> None:
    rng = np.random.default_rng(42)
    for _ in range(100):
        raw, correction = rng.normal(size=(2, 3))
        imu = np.r_[10., np.zeros(5), (raw + correction) * [1, -1, -1], np.zeros(3)]
        history = np.array([[9.9, 9.9, *correction]])
        old_imu, old_history = imu.copy(), history.copy()
        value, removed, stamp = MODULE.gyro_frd_from_recorded_imu(
            imu, history, gyro_source="attitude_minus_ahrs")
        np.testing.assert_allclose(value, raw, rtol=0, atol=1e-15)
        np.testing.assert_array_equal(removed, correction)
        assert stamp == 9.9
        np.testing.assert_array_equal(imu, old_imu)
        np.testing.assert_array_equal(history, old_history)


@pytest.mark.parametrize("times", ([10.01], [9.0], []))
def test_future_stale_and_missing_history_never_fill_a_bias(times: list[float]) -> None:
    imu = np.r_[10., np.zeros(5), .1, .2, .3, np.zeros(3)]
    history = np.array([[t, t, 1., 2., 3.] for t in times])
    value, drift, stamp = MODULE.gyro_frd_from_recorded_imu(imu, history, gyro_source="attitude_minus_ahrs")
    np.testing.assert_array_equal(value, [.1, -.2, -.3])
    np.testing.assert_array_equal(drift, [0, 0, 0])
    assert stamp is None


def test_newer_ahrs_cannot_change_an_older_held_imu_measurement() -> None:
    imu = np.r_[10., np.zeros(5), .1, .2, .3, np.zeros(3)]
    history = np.array([[9.9, 9.9, .01, .02, .03], [10.01, 10.01, 1., 2., 3.]])
    value, drift, stamp = MODULE.gyro_frd_from_recorded_imu(imu, history, gyro_source="attitude_minus_ahrs")
    np.testing.assert_allclose(value, [.09, -.22, -.33])
    assert stamp == 9.9
    prior, _, _ = MODULE.gyro_frd_from_recorded_imu(imu, history, gyro_source="attitude")
    np.testing.assert_array_equal(prior, [.1, -.2, -.3])


def test_raw_imu_uses_only_received_sensor_values_with_frame_conversion() -> None:
    history = np.zeros((3, 12))
    history[:, 0] = [1., 1.5, 2.]
    history[:, 6:9] = [[.1, .2, .3], [.4, .5, .6], [10., 20., 30.]]
    original = history.copy()
    value, receipt = MODULE.gyro_frd_from_recorded_raw_imu(history, 1.75)
    np.testing.assert_array_equal(value, [.4, -.5, -.6])
    assert receipt == 1.5
    np.testing.assert_array_equal(history, original)


@pytest.mark.parametrize("history", (np.empty((0, 12)), np.array([[1., *np.zeros(11)]])))
def test_raw_imu_does_not_backfill_unobserved_past(history: np.ndarray) -> None:
    value, receipt = MODULE.gyro_frd_from_recorded_raw_imu(history, .5)
    assert value is None and receipt is None

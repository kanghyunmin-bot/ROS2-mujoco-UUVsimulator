"""Independent synthetic timing, missing-data and noise checks."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rosbag_trend_math import (
    clock_diagnostics,
    interpolate_valid,
    metrics,
    observation_envelope,
    point_velocity,
    sample_envelope,
    select_phase,
    trend,
)


def waveform(t):
    return np.sin(0.19 * t * t) + 0.3 * np.sin(2.4 * t)


def phase_fixture(real):
    t = np.arange(0, 30, 0.05)
    return select_phase(
        t, real, t, waveform(t), train=(2, 12), validation=(13, 23), noise_sigma=0.005
    )


def test_known_phase_is_recovered_without_scaling():
    t = np.arange(0, 30, 0.05)
    result, baseline, selected, _, mask = phase_fixture(waveform(t - 0.17))
    assert result["accepted"]
    assert result["selected_lag_s"] == pytest.approx(0.17, abs=0.01)
    assert metrics(waveform(t - 0.17)[mask], selected[mask])["rmse"] < 0.02
    assert metrics(waveform(t - 0.17)[mask], baseline[mask])["rmse"] > 0.1


def test_validation_rejects_a_train_only_phase_improvement():
    t = np.arange(0, 30, 0.05)
    real = np.where(t < 12.5, waveform(t - 0.2), waveform(t))
    result, *_ = phase_fixture(real)
    assert result["candidate_lag_s"] == pytest.approx(0.2, abs=0.01)
    assert not result["accepted"]
    assert result["selected_lag_s"] == 0


def test_future_test_data_cannot_change_selected_phase():
    t = np.arange(0, 30, 0.05)
    real = waveform(t - 0.17)
    before, *_ = phase_fixture(real)
    real[t >= 23] += 100 * np.sin(t[t >= 23])
    after, *_ = phase_fixture(real)
    assert before == after


def test_gaps_invalid_measurements_and_extrapolation_remain_missing():
    t = np.array([0, 1, 2, 5.0])
    value = np.array([0, 1, 2, 5.0])
    query = np.array([-0.1, 0, 0.5, 1, 1.5, 2, 3, 5, 5.1])
    out = interpolate_valid(
        t, value, query, max_gap_s=1.1, valid=[True, False, True, True]
    )
    assert np.array_equal(
        np.isfinite(out), [False, True, False, False, False, True, False, True, False]
    )
    np.testing.assert_equal(out[[1, 5, 7]], [0, 2, 5])


@pytest.mark.parametrize("t", [[0, 0, 1], [0, 2, 1], [0, float("nan"), 2]])
def test_corrupt_source_clock_is_rejected(t):
    with pytest.raises(ValueError):
        interpolate_valid(t, [1, 2, 3], [0.5], max_gap_s=1)


def test_centered_trend_does_not_hide_gaps():
    x = np.arange(15, dtype=float)
    x[7] = np.nan
    y = trend(x, 3)
    assert np.all(np.isnan(y[[0, 6, 7, 8, 14]]))
    assert y[3] == 3


def test_noise_draws_are_reproducible_and_preserve_covariance():
    config = {"ar1_rho": 0.5, "covariance": [[0.01, 0.003], [0.003, 0.04]]}
    values = sample_envelope(config, 60000, 17)
    np.testing.assert_array_equal(values, sample_envelope(config, 60000, 17))
    np.testing.assert_allclose(np.cov(values.T), config["covariance"], atol=0.001)
    assert np.max(abs(values.mean(0))) < 0.003
    assert np.corrcoef(values[1:, 0], values[:-1, 0])[0, 1] == pytest.approx(
        0.5, abs=0.02
    )


def test_noise_estimate_is_centered_not_gravity_or_constant_bias():
    t = np.arange(1000) * 0.1
    v = sample_envelope({"ar1_rho": 0, "covariance": [[0.01]]}, len(t), 1) + 8.9
    result = observation_envelope(t, v, max_gap_s=0.2)
    assert result["std"][0] == pytest.approx(0.1, abs=0.01)
    assert result["robust_difference_sigma"][0] == pytest.approx(0.1, abs=0.015)
    assert result["median"][0] == pytest.approx(8.9, abs=0.02)


def test_lever_arm_matches_rotating_rigid_body():
    np.testing.assert_allclose(
        point_velocity([1, 0, 0], [0, 0, 2], [0.1, 0.2, 0]), [0.6, 0.2, 0]
    )


def test_noise_is_not_subtracted_from_rmse():
    result = metrics(np.zeros(10), np.ones(10), noise_sigma=2)
    assert result["rmse"] == 1
    assert result["rmse_over_stationary_sigma"] == 0.5


def test_clock_audit_preserves_large_header_offset_and_duplicate_headers():
    receipt = 1_775_133_981_020_936_721 + np.arange(4, dtype=np.int64) * 500_000_000
    header = receipt - 4_294_968_500_000
    header[2] = header[1]
    result = clock_diagnostics(np.column_stack((receipt, header)))
    assert result["receipt_rate_median_hz"] == 2
    assert result["nonincreasing_receipt_count"] == 0
    assert result["nonincreasing_header_count"] == 1
    assert result["receipt_minus_header_s"]["median"] == pytest.approx(4294.9685)


@pytest.mark.parametrize("covariance", [[[1, 1], [0, 1]], [[1, 2], [2, 1]], [[np.nan]]])
def test_invalid_noise_covariance_is_rejected(covariance):
    with pytest.raises(ValueError):
        sample_envelope({"ar1_rho": 0.5, "covariance": covariance}, 10, 0)


def test_high_correlation_does_not_conceal_amplitude_error():
    real = np.linspace(-1, 1, 200)
    result = metrics(real, 5 * real)
    assert result["correlation"] == pytest.approx(1)
    assert result["rmse"] > 2


def test_phase_search_respects_nonintegral_bounds():
    t = np.arange(0, 30, 0.05)
    result, *_ = select_phase(
        t,
        waveform(t - 0.1),
        t,
        waveform(t),
        train=(2, 12),
        validation=(13, 23),
        max_lag_s=0.305,
        lag_step_s=0.02,
    )
    assert all(abs(lag) <= 0.305 + 1e-12 for lag, _ in result["train_loss_by_lag"])
    assert result["split_margin_s"] >= 0.455 - 1e-12

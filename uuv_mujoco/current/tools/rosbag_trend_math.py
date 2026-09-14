"""Offline, noise-aware comparisons with frozen temporal validation.

No function modifies timestamps in the source recording or vehicle controls.
"""

from __future__ import annotations

import numpy as np


def interpolate_valid(t, values, query, *, max_gap_s, valid=None):
    """Interpolate bounded valid neighbours; never extrapolate or bridge dropouts."""
    t, values, query = (np.asarray(x, dtype=float) for x in (t, values, query))
    if (
        t.ndim != 1
        or len(t) < 2
        or not np.all(np.isfinite(t))
        or np.any(np.diff(t) <= 0)
    ):
        raise ValueError("Source times must be finite, unique and increasing")
    if (
        values.ndim not in (1, 2)
        or values.shape[0] != len(t)
        or query.ndim != 1
        or not np.isfinite(max_gap_s)
        or max_gap_s <= 0
        or not np.all(np.isfinite(query))
    ):
        raise ValueError("Invalid interpolation shape, query or gap")
    scalar = values.ndim == 1
    values = values[:, None] if scalar else values
    if values.ndim != 2:
        raise ValueError("Values must be N or NxD")
    ok = np.all(np.isfinite(values), axis=1)
    if valid is not None:
        ok &= np.asarray(valid, dtype=bool)
    hi = np.clip(np.searchsorted(t, query, side="left"), 1, len(t) - 1)
    lo = hi - 1
    weight = (query - t[lo]) / (t[hi] - t[lo])
    out = values[lo] * (1 - weight[:, None]) + values[hi] * weight[:, None]
    good = (query >= t[0]) & (query <= t[-1]) & ok[lo] & ok[hi]
    good &= (t[hi] - t[lo]) <= max_gap_s
    # An exact valid measurement does not require its neighbour to be valid.
    exact = np.clip(np.searchsorted(t, query), 0, len(t) - 1)
    hit = (abs(query - t[exact]) <= 1e-10) & ok[exact]
    out[hit] = values[exact[hit]]
    good |= hit
    out[~good] = np.nan
    return out[:, 0] if scalar else out


def trend(values, samples):
    """Centered mean with complete support; preserve invalid gaps and edge loss."""
    values = np.asarray(values, dtype=float)
    if values.ndim != 1 or samples < 1 or samples % 2 != 1:
        raise ValueError("Trend requires a 1D series and an odd positive window")
    if samples > len(values):
        return np.full_like(values, np.nan)
    finite = np.isfinite(values)
    count = np.convolve(finite.astype(float), np.ones(samples), mode="same")
    out = np.convolve(
        np.where(finite, values, 0), np.ones(samples) / samples, mode="same"
    )
    out[count < samples] = np.nan
    return out


def metrics(reference, prediction, *, noise_sigma=None):
    """Return raw bias/RMSE/correlation; noise ratio is descriptive, not a CI."""
    reference, prediction = np.asarray(reference), np.asarray(prediction)
    if reference.shape != prediction.shape or reference.ndim != 1:
        raise ValueError("Metric signals must have the same 1D shape")
    good = np.isfinite(reference) & np.isfinite(prediction)
    if sum(good) < 3:
        return {"n": int(sum(good)), "rmse": None, "correlation": None}
    real, sim = reference[good], prediction[good]
    residual = sim - real
    rmse = float(np.sqrt(np.mean(residual**2)))
    scale = float(np.quantile(real, 0.95) - np.quantile(real, 0.05))
    return {
        "n": int(sum(good)),
        "rmse": rmse,
        "bias": float(np.mean(residual)),
        "centered_rmse": float(np.std(residual)),
        "p95_abs_error": float(np.quantile(abs(residual), 0.95)),
        "reference_p05_p95_span": scale,
        "rmse_over_reference_span": rmse / scale if scale > 1e-10 else None,
        "rmse_over_stationary_sigma": rmse / noise_sigma
        if noise_sigma and noise_sigma > 0
        else None,
        "correlation": float(np.corrcoef(real, sim)[0, 1])
        if min(np.std(real), np.std(sim)) > 1e-10
        else None,
    }


def select_phase(
    t,
    reference,
    sim_t,
    prediction,
    *,
    train,
    validation,
    max_lag_s=0.3,
    lag_step_s=0.01,
    max_gap_s=0.15,
    smooth_samples=7,
    noise_sigma=0.0,
):
    """Fit one bounded offset on train; accept only if validation also improves.

    A positive lag compares real(t) with sim(t-lag). It is an effective comparison
    offset, not identified hardware latency. No scale or time warp is fitted.
    All candidates use exactly the same supported timestamps.
    """
    if not np.isfinite(max_lag_s) or not 0 < lag_step_s <= max_lag_s:
        raise ValueError("Invalid lag bounds")
    if not train[0] < train[1] <= validation[0] < validation[1]:
        raise ValueError("Train and validation must be ordered and disjoint")
    t = np.asarray(t)
    if (
        t.ndim != 1
        or len(t) < 2
        or not np.all(np.isfinite(t))
        or np.any(np.diff(t) <= 0)
        or not np.isfinite(noise_sigma)
        or noise_sigma < 0
    ):
        raise ValueError("Invalid comparison clock or noise scale")
    lags = np.unique(
        np.r_[np.arange(-max_lag_s, max_lag_s + 1e-12, lag_step_s), 0.0, max_lag_s]
    )
    lags[np.abs(lags) < 1e-12] = 0.0
    lags = np.unique(lags)
    candidates = np.array(
        [
            interpolate_valid(sim_t, prediction, t - lag, max_gap_s=max_gap_s)
            for lag in lags
        ]
    )
    reference_trend = trend(reference, smooth_samples)
    smooth = np.array([trend(row, smooth_samples) for row in candidates])
    support = np.isfinite(reference_trend) & np.all(np.isfinite(smooth), axis=0)
    # Erode split boundaries so centered smoothing/lag support cannot read the
    # next split while fitting. The reported test is never used for selection.
    margin = (smooth_samples // 2) * float(np.max(np.diff(t))) + max_lag_s
    fit = support & (t >= train[0] + margin) & (t < train[1] - margin)
    val = support & (t >= validation[0] + margin) & (t < validation[1] - margin)
    zero = int(np.flatnonzero(lags == 0)[0])
    if min(sum(fit), sum(val)) < 20:
        raise ValueError("Insufficient common train/validation support")
    amplitude = np.ptp(reference_trend[fit])
    scale = max(float(noise_sigma), 0.02 * amplitude, 1e-8)
    residual = (smooth[:, fit] - reference_trend[fit]) / scale
    # Huber loss limits isolated spikes without erasing them from raw metrics.
    loss = np.mean(
        np.where(abs(residual) <= 2, 0.5 * residual**2, 2 * abs(residual) - 2), axis=1
    )
    best = int(np.argmin(loss))
    baseline = metrics(reference_trend[val], smooth[zero, val])
    candidate = metrics(reference_trend[val], smooth[best, val])
    raw_base = metrics(np.asarray(reference)[val], candidates[zero, val])
    raw_new = metrics(np.asarray(reference)[val], candidates[best, val])
    reasons = []
    if amplitude < max(6 * noise_sigma, 1e-6):
        reasons.append("insufficient_excitation")
    if abs(lags[best]) >= max_lag_s - lag_step_s / 2:
        reasons.append("optimum_at_search_boundary")
    if candidate["rmse"] >= 0.98 * baseline["rmse"]:
        reasons.append("validation_trend_improvement_below_2_percent")
    if raw_new["rmse"] > 1.02 * raw_base["rmse"]:
        reasons.append("validation_raw_rmse_worsened")
    selected = zero if reasons else best
    return (
        {
            "candidate_lag_s": float(lags[best]),
            "selected_lag_s": float(lags[selected]),
            "accepted": not reasons,
            "reasons": reasons,
            "train_window_s": list(train),
            "validation_window_s": list(validation),
            "train_n": int(sum(fit)),
            "validation_n": int(sum(val)),
            "split_margin_s": margin,
            "validation_baseline_trend": baseline,
            "validation_candidate_trend": candidate,
            "train_loss_by_lag": list(zip(lags.tolist(), loss.tolist())),
            "offset_interpretation": "real(t) vs sim(t-lag); not hardware latency",
        },
        candidates[zero],
        candidates[selected],
        candidates[best],
        support,
    )


def observation_envelope(t, values, *, max_gap_s):
    """Estimate a low-motion output envelope, including residual motion/filtering."""
    t, values = np.asarray(t), np.asarray(values, dtype=float)
    if values.ndim == 1:
        values = values[:, None]
    if (
        len(t) < 20
        or t.ndim != 1
        or values.ndim != 2
        or values.shape[0] != len(t)
        or not np.all(np.isfinite(t))
        or not np.all(np.isfinite(values))
        or np.any(np.diff(t) <= 0)
        or not np.isfinite(max_gap_s)
        or max_gap_s <= 0
    ):
        raise ValueError("Need >=20 finite, ordered envelope measurements")
    centered = values - np.median(values, axis=0)
    pairs = np.diff(t) <= max_gap_s
    if sum(pairs) < 15:
        raise ValueError("Insufficient adjacent low-motion pairs")
    diff = np.diff(values, axis=0)[pairs]
    robust = (
        1.4826 * np.median(abs(diff - np.median(diff, axis=0)), axis=0) / np.sqrt(2)
    )
    rho = np.sum(centered[:-1][pairs] * centered[1:][pairs], axis=0) / np.maximum(
        np.sum(centered[:-1][pairs] ** 2, axis=0), 1e-20
    )
    # One common AR(1) coefficient preserves the cross-axis stationary covariance.
    common_rho = float(np.clip(np.mean(rho), 0, 0.98))
    return {
        "n": len(t),
        "dt_median_s": float(np.median(np.diff(t))),
        "median": np.median(values, axis=0).tolist(),
        "std": np.std(values, axis=0, ddof=1).tolist(),
        "robust_difference_sigma": robust.tolist(),
        "covariance": np.atleast_2d(np.cov(values, rowvar=False)).tolist(),
        "lag1_by_axis": rho.tolist(),
        "ar1_rho": common_rho,
        "scope": "empirical low-motion output envelope; not identified raw hardware noise",
    }


def sample_envelope(envelope, count, seed):
    """Draw zero-mean correlated observations at the fitted sample interval."""
    covariance = np.asarray(envelope["covariance"], dtype=float)
    rho = float(envelope["ar1_rho"])
    if (
        covariance.ndim != 2
        or covariance.shape[0] != covariance.shape[1]
        or covariance.shape[0] == 0
        or not np.all(np.isfinite(covariance))
        or not np.allclose(covariance, covariance.T, rtol=1e-10, atol=1e-12)
    ):
        raise ValueError("Invalid envelope covariance")
    eigenvalues, vectors = np.linalg.eigh(covariance)
    if np.min(eigenvalues) < -1e-12 or not 0 <= rho < 1 or count < 1:
        raise ValueError("Non-PSD covariance, unstable AR model or empty request")
    root = vectors @ np.diag(np.sqrt(np.maximum(eigenvalues, 0)))
    noise = np.random.default_rng(seed).normal(size=(count, len(eigenvalues))) @ root.T
    result = noise.copy()
    for i in range(1, count):
        result[i] = rho * result[i - 1] + np.sqrt(1 - rho**2) * noise[i]
    return result


def point_velocity(velocity_reference, omega_body, lever_body):
    """Rigid-body sensor velocity [m/s] with lever arm [m] and omega [rad/s]."""
    return np.asarray(velocity_reference) + np.cross(omega_body, lever_body)


def clock_diagnostics(record_header_ns):
    """Audit receipt/header clocks [ns] without treating their difference as latency."""
    stamps = np.asarray(record_header_ns)
    if stamps.ndim != 2 or stamps.shape[1] != 2 or len(stamps) < 2:
        raise ValueError("Clock audit requires at least two paired timestamps")
    if not np.issubdtype(stamps.dtype, np.integer):
        raise ValueError("Preserve integer nanoseconds for clock diagnostics")
    intervals = np.diff(stamps, axis=0) / 1e9
    age = (stamps[:, 0] - stamps[:, 1]) / 1e9
    positive = intervals[:, 0][intervals[:, 0] > 0]
    median = float(np.median(positive)) if len(positive) else None
    return {
        "n": len(stamps),
        "receipt_dt_median_s": median,
        "receipt_rate_median_hz": 1 / median if median else None,
        "nonincreasing_receipt_count": int(sum(intervals[:, 0] <= 0)),
        "nonincreasing_header_count": int(sum(intervals[:, 1] <= 0)),
        "receipt_minus_header_s": dict(
            zip(
                ("min", "median", "p95", "max"),
                np.quantile(age, [0, 0.5, 0.95, 1]).tolist(),
            )
        ),
        "interpretation": "May contain clock offset, queueing and transport; not identified sensor latency.",
    }

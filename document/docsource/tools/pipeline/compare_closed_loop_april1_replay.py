from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_april1_real_bags import first_string_time_in_bag, read_bag, scalar_stats, vector_stats  # noqa: E402


DEFAULT_REAL_BAG = Path(
    "real_robot_ros_bag/extracted_2026_04_01/"
    "bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_0.db3"
)
DEFAULT_OUT = Path("document/docsource/runs/closed_loop/closed_loop_april1_rc_replay_compare")


def resolve_db3(path: Path) -> Path:
    if path.is_file() and path.suffix == ".db3":
        return path
    if path.is_dir():
        candidates = sorted(path.glob("*.db3"))
        if candidates:
            return candidates[0]
        candidates = sorted(path.glob("**/*.db3"))
        if candidates:
            return candidates[0]
    raise FileNotFoundError(f"No .db3 bag file found at {path}")


def first_array(data, keys: list[str], dims: int | None = None) -> tuple[np.ndarray, np.ndarray, str]:
    for key in keys:
        try:
            t, v = data.array(key, dims)
        except ValueError:
            if dims is not None:
                raise
            raw = data.series_v.get(key, [])
            t = np.asarray(data.series_t.get(key, []), dtype=float)
            max_len = max((len(row) if hasattr(row, "__len__") else 1) for row in raw) if raw else 0
            v = np.full((len(raw), max_len), np.nan, dtype=float)
            for row_idx, row in enumerate(raw):
                if hasattr(row, "__len__") and not isinstance(row, (str, bytes)):
                    values = [float(item) for item in row]
                else:
                    values = [float(row)]
                v[row_idx, : len(values)] = values
        if v.size:
            return t, v, key
    empty_v = np.empty((0, dims), dtype=float) if dims is not None else np.empty(0, dtype=float)
    return np.empty(0, dtype=float), empty_v, ""


def first_string_time(db_path: Path, topic: str, value: str) -> float | None:
    try:
        return first_string_time_in_bag(db_path, topic, value)
    except Exception:
        return None


def rc_start_alignment(
    real,
    sim,
    real_db: Path,
    sim_db: Path,
    topic_key: str,
    *,
    real_start_offset_s: float = 0.0,
) -> dict[str, Any]:
    real_t, _, real_key = first_array(real, [topic_key])
    if real_t.size and real_start_offset_s > 0.0:
        real_t = real_t[real_t >= float(real_start_offset_s) - 1.0e-9]
    sim_phase_label = "closed_loop_replay"
    sim_phase_start = first_string_time(sim_db, "/measurement/phase", sim_phase_label)
    if sim_phase_start is None:
        sim_phase_label = "rcout_replay"
        sim_phase_start = first_string_time(sim_db, "/measurement/phase", sim_phase_label)
    if sim_phase_start is not None:
        sim_t = np.asarray([float(sim_phase_start)], dtype=float)
        sim_key = f"/measurement/phase:{sim_phase_label}:first"
        method = "real_rc_start_to_sim_replay_phase"
    else:
        sim_t, _, sim_key = first_array(sim, [topic_key])
        method = "real_rc_start_to_sim_rc_start"
    if real_t.size == 0 or sim_t.size == 0:
        return {
            "status": "unavailable",
            "method": method,
            "topic_key": topic_key,
            "real_key": real_key,
            "sim_key": sim_key,
            "real_start_offset_s": float(real_start_offset_s),
            "real_start_s": None,
            "sim_start_s": None,
            "sim_time_shift_s": 0.0,
        }
    real_start = float(real_t[0])
    sim_start = float(sim_t[0])
    return {
        "status": "applied",
        "method": method,
        "topic_key": topic_key,
        "real_key": real_key,
        "sim_key": sim_key,
        "real_start_offset_s": float(real_start_offset_s),
        "real_start_s": real_start,
        "sim_start_s": sim_start,
        "sim_time_shift_s": real_start - sim_start,
    }


def fit_gain_offset(x: np.ndarray, y: np.ndarray) -> dict[str, Any]:
    x = np.asarray(x, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    finite = np.isfinite(x) & np.isfinite(y)
    x = x[finite]
    y = y[finite]
    if x.size < 12 or np.std(x) < 1e-12 or np.std(y) < 1e-12:
        return {"count": int(x.size), "gain": None, "offset": None, "r2": None}
    A = np.column_stack([np.ones_like(x), x])
    beta, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ beta
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return {
        "count": int(x.size),
        "offset": float(beta[0]),
        "gain": float(beta[1]),
        "r2": float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else None,
        "residual_std": float(np.std(y - pred)),
    }


def compare_scalar_with_lag(
    real_t: np.ndarray,
    real_y: np.ndarray,
    sim_t: np.ndarray,
    sim_y: np.ndarray,
    *,
    crop_start_s: float,
    crop_end_s: float | None,
    lag_min_s: float,
    lag_max_s: float,
    lag_step_s: float,
    remove_median_offset: bool = False,
) -> dict[str, Any]:
    real_t = np.asarray(real_t, dtype=float).reshape(-1)
    real_y = np.asarray(real_y, dtype=float).reshape(-1)
    sim_t = np.asarray(sim_t, dtype=float).reshape(-1)
    sim_y = np.asarray(sim_y, dtype=float).reshape(-1)
    if real_t.size < 12 or sim_t.size < 12:
        return {"count": 0}
    best: dict[str, Any] | None = None
    for lag in np.arange(lag_min_s, lag_max_s + 1e-9, lag_step_s):
        query_t = real_t - float(lag)
        mask = (
            (real_t >= crop_start_s)
            & (query_t >= sim_t[0])
            & (query_t <= sim_t[-1])
            & np.isfinite(real_y)
        )
        if crop_end_s is not None:
            mask &= real_t <= float(crop_end_s)
        if np.sum(mask) < 12:
            continue
        r = real_y[mask]
        s = np.interp(query_t[mask], sim_t, sim_y)
        finite = np.isfinite(r) & np.isfinite(s)
        r = r[finite]
        s = s[finite]
        if r.size < 12:
            continue
        residual = s - r
        offset = 0.0
        if remove_median_offset:
            offset = float(np.median(residual))
            residual = residual - offset
        corr = None
        if np.std(r) > 1e-12 and np.std(s) > 1e-12:
            corr = float(np.corrcoef(r, s)[0, 1])
        real_stats = scalar_stats(real_t[mask][finite], r)
        sim_stats = scalar_stats(real_t[mask][finite], s)
        real_rms = float(real_stats.get("rms", 0.0) or 0.0)
        sim_rms = float(sim_stats.get("rms", 0.0) or 0.0)
        scale = max(real_rms, float(np.std(r)), float(np.mean(np.abs(r))), 1.0e-6)
        rms_ratio = sim_rms / max(real_rms, 1.0e-9) if real_rms > 1.0e-12 else None
        corr_score = 0.5 if corr is None else max(0.0, min(1.0, 0.5 * (corr + 1.0)))
        rmse_score = float(np.exp(-float(np.sqrt(np.mean(residual * residual))) / scale))
        if rms_ratio is None or rms_ratio <= 0.0:
            ratio_score = 0.0
        else:
            ratio_score = float(np.exp(-abs(np.log(rms_ratio))))
        bias_score = float(np.exp(-abs(float(np.mean(residual))) / scale))
        lag_score = float(np.exp(-abs(float(lag)) / max(abs(lag_max_s), 1.0e-6)))
        confidence_score = 100.0 * (
            0.30 * rmse_score
            + 0.30 * corr_score
            + 0.20 * ratio_score
            + 0.10 * bias_score
            + 0.10 * lag_score
        )
        candidate = {
            "count": int(r.size),
            "lag_s": float(lag),
            "rmse": float(np.sqrt(np.mean(residual * residual))),
            "mae": float(np.mean(np.abs(residual))),
            "bias": float(np.mean(residual)),
            "residual_std": float(np.std(residual)),
            "correlation": corr,
            "removed_median_offset": offset,
            "real": real_stats,
            "sim": sim_stats,
            "real_rms": real_rms,
            "sim_rms": sim_rms,
            "sim_real_rms_ratio": rms_ratio,
            "confidence_score": float(confidence_score),
            "gain_fit_real_from_sim": fit_gain_offset(s, r),
        }
        if best is None:
            best = candidate
            continue
        # Use signed correlation for lag selection. An inverted axis can have a
        # large absolute correlation, but it must not be treated as a good
        # overlay candidate.
        best_corr = -1.0 if best["correlation"] is None else float(best["correlation"])
        cand_corr = -1.0 if candidate["correlation"] is None else float(candidate["correlation"])
        if cand_corr > best_corr + 1e-9 or (
            abs(cand_corr - best_corr) <= 1e-9 and candidate["rmse"] < best["rmse"]
        ):
            best = candidate
    return best or {"count": 0}


def compare_vector(
    real_t: np.ndarray,
    real_v: np.ndarray,
    sim_t: np.ndarray,
    sim_v: np.ndarray,
    *,
    names: tuple[str, str, str],
    crop_start_s: float,
    crop_end_s: float | None,
) -> dict[str, Any]:
    if real_v.size == 0 or sim_v.size == 0:
        return {"count": 0}
    return {
        name: compare_scalar_with_lag(
            real_t,
            real_v[:, idx],
            sim_t,
            sim_v[:, idx],
            crop_start_s=crop_start_s,
            crop_end_s=crop_end_s,
            lag_min_s=-2.0,
            lag_max_s=2.0,
            lag_step_s=0.05,
        )
        for idx, name in enumerate(names)
    }


def compare_matrix_columns(
    real_t: np.ndarray,
    real_v: np.ndarray,
    sim_t: np.ndarray,
    sim_v: np.ndarray,
    *,
    names: tuple[str, ...],
    crop_start_s: float,
    crop_end_s: float | None,
    lag_min_s: float = -0.5,
    lag_max_s: float = 0.5,
    lag_step_s: float = 0.02,
) -> dict[str, Any]:
    if real_v.size == 0 or sim_v.size == 0:
        return {"count": 0}
    width = min(real_v.shape[1], sim_v.shape[1], len(names))
    return {
        names[idx]: compare_scalar_with_lag(
            real_t,
            real_v[:, idx],
            sim_t,
            sim_v[:, idx],
            crop_start_s=crop_start_s,
            crop_end_s=crop_end_s,
            lag_min_s=lag_min_s,
            lag_max_s=lag_max_s,
            lag_step_s=lag_step_s,
        )
        for idx in range(width)
    }


def rcout_pwm_to_norm(values: np.ndarray) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return arr.reshape((0, 0))
    out = np.full_like(arr, np.nan, dtype=float)
    valid = (arr > 0.0) & (arr < 65535.0)
    out[valid] = np.clip((arr[valid] - 1500.0) / 400.0, -1.0, 1.0)
    return out


def rcout_vertical_axes(values: np.ndarray) -> np.ndarray:
    norm = rcout_pwm_to_norm(values)
    if norm.size == 0 or norm.shape[1] < 8:
        return np.empty((0, 3), dtype=float)
    signs = np.asarray([-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0], dtype=float)
    cmd = norm[:, :8] * signs
    ver_rf = cmd[:, 4]
    ver_lf = cmd[:, 5]
    ver_rr = cmd[:, 6]
    ver_lr = cmd[:, 7]
    vertical_common = np.nanmean(np.column_stack([ver_lf, ver_lr, ver_rf, ver_rr]), axis=1)
    vertical_roll_diff = 0.5 * ((ver_lf + ver_lr) - (ver_rf + ver_rr))
    vertical_pitch_diff = 0.5 * ((ver_lf + ver_rf) - (ver_lr + ver_rr))
    return np.column_stack([vertical_common, vertical_roll_diff, vertical_pitch_diff])


def velocity_gyro_score_no_depth(
    dvl_metrics: dict[str, Any],
    gyro_metrics: dict[str, Any],
) -> dict[str, Any]:
    axes = [
        ("dvl_x", dvl_metrics.get("x", {}), 1.0),
        ("dvl_y", dvl_metrics.get("y", {}), 1.0),
        ("dvl_z", dvl_metrics.get("z", {}), 1.0),
        ("gyro_x", gyro_metrics.get("x", {}), 0.7),
        ("gyro_y", gyro_metrics.get("y", {}), 0.7),
        ("gyro_z", gyro_metrics.get("z", {}), 0.7),
    ]
    rows: dict[str, Any] = {}
    weighted_norm_rmse = 0.0
    weighted_corr = 0.0
    weight_sum = 0.0
    for name, metrics, weight in axes:
        count = int(metrics.get("count", 0) or 0)
        real = metrics.get("real", {}) if isinstance(metrics.get("real", {}), dict) else {}
        sim = metrics.get("sim", {}) if isinstance(metrics.get("sim", {}), dict) else {}
        real_rms = float(real.get("rms", 0.0) or 0.0)
        sim_rms = float(sim.get("rms", 0.0) or 0.0)
        rmse = float(metrics.get("rmse", 0.0) or 0.0)
        corr_raw = metrics.get("correlation")
        corr = 0.0 if corr_raw is None else float(corr_raw)
        norm_rmse = rmse / max(real_rms, 1.0e-6)
        rms_ratio_real_over_sim = real_rms / max(sim_rms, 1.0e-6)
        rows[name] = {
            "count": count,
            "rmse": rmse,
            "real_rms": real_rms,
            "sim_rms": sim_rms,
            "normalized_rmse": norm_rmse,
            "correlation": corr_raw,
            "rms_ratio_real_over_sim": rms_ratio_real_over_sim,
        }
        if count > 0:
            weighted_norm_rmse += float(weight) * norm_rmse
            weighted_corr += float(weight) * corr
            weight_sum += float(weight)
    score = weighted_norm_rmse / max(weight_sum, 1.0e-9)
    return {
        "definition": "Depth-excluded score over DVL xyz and IMU gyro xyz. Lower normalized_rmse is better; higher mean_correlation is better.",
        "weighted_normalized_rmse": float(score),
        "weighted_mean_correlation": float(weighted_corr / max(weight_sum, 1.0e-9)),
        "axis_metrics": rows,
    }


def _clamp_percent(value: float) -> float:
    return float(max(0.0, min(100.0, value)))


def operational_confidence_no_depth(
    dvl_metrics: dict[str, Any],
    gyro_metrics: dict[str, Any],
    normalized_score: dict[str, Any],
) -> dict[str, Any]:
    """Task-oriented confidence for controller development.

    The normalized-RMSE score is useful for exact waveform identification, but it
    unfairly collapses near-zero stable roll/pitch axes. This score separates
    phase, RMS-amplitude and bounded-error behavior, and treats roll/pitch gyro
    primarily as a stability-envelope check.
    """

    axis_metrics = normalized_score.get("axis_metrics", {}) if isinstance(normalized_score, dict) else {}
    tolerances = {
        "dvl_x": 0.15,
        "dvl_y": 0.07,
        "dvl_z": 0.07,
        "gyro_x": 0.10,
        "gyro_y": 0.10,
        "gyro_z": 0.35,
    }
    source = {
        "dvl_x": dvl_metrics.get("x", {}),
        "dvl_y": dvl_metrics.get("y", {}),
        "dvl_z": dvl_metrics.get("z", {}),
        "gyro_x": gyro_metrics.get("x", {}),
        "gyro_y": gyro_metrics.get("y", {}),
        "gyro_z": gyro_metrics.get("z", {}),
    }
    rows: dict[str, Any] = {}
    response_axes = []
    stability_axes = []
    for name, metrics in source.items():
        norm = axis_metrics.get(name, {}) if isinstance(axis_metrics, dict) else {}
        count = int(metrics.get("count", 0) or 0)
        corr_raw = metrics.get("correlation")
        phase_percent = 50.0 if corr_raw is None else _clamp_percent((float(corr_raw) + 1.0) * 50.0)
        real = metrics.get("real", {}) if isinstance(metrics.get("real", {}), dict) else {}
        sim = metrics.get("sim", {}) if isinstance(metrics.get("sim", {}), dict) else {}
        real_rms = float(real.get("rms", 0.0) or 0.0)
        sim_rms = float(sim.get("rms", 0.0) or 0.0)
        if real_rms <= 1.0e-9 or sim_rms <= 1.0e-9:
            amplitude_percent = 0.0
        else:
            ratio = real_rms / sim_rms
            amplitude_percent = _clamp_percent(min(ratio, 1.0 / ratio) * 100.0)
        rmse = float(metrics.get("rmse", 0.0) or 0.0)
        error_band_percent = _clamp_percent((1.0 - rmse / max(tolerances[name], 1.0e-9)) * 100.0)
        if name in {"gyro_x", "gyro_y"}:
            real_p95_abs = max(abs(float(real.get("p05", 0.0) or 0.0)), abs(float(real.get("p95", 0.0) or 0.0)))
            sim_p95_abs = max(abs(float(sim.get("p05", 0.0) or 0.0)), abs(float(sim.get("p95", 0.0) or 0.0)))
            stability_percent = 0.5 * (
                _clamp_percent((1.0 - real_p95_abs / 0.15) * 100.0)
                + _clamp_percent((1.0 - sim_p95_abs / 0.15) * 100.0)
            )
            operational = 0.70 * stability_percent + 0.20 * error_band_percent + 0.10 * phase_percent
            stability_axes.append(operational)
        else:
            stability_percent = None
            operational = 0.45 * phase_percent + 0.35 * amplitude_percent + 0.20 * error_band_percent
            response_axes.append(operational)
        rows[name] = {
            "count": count,
            "rmse": rmse,
            "correlation": corr_raw,
            "normalized_rmse": norm.get("normalized_rmse"),
            "phase_percent": float(phase_percent),
            "amplitude_percent": float(amplitude_percent),
            "error_band_percent": float(error_band_percent),
            "stability_percent": None if stability_percent is None else float(stability_percent),
            "operational_confidence_percent": float(_clamp_percent(operational)),
        }

    all_scores = [row["operational_confidence_percent"] for row in rows.values() if row.get("count", 0) > 0]
    return {
        "definition": (
            "Controller-development confidence. DVL xyz and yaw-rate combine phase, RMS amplitude, and "
            "bounded RMSE; roll/pitch rates are evaluated mainly as a stability envelope. This is not a "
            "system-identification grade truth score."
        ),
        "average_response_confidence_percent": float(np.mean(response_axes)) if response_axes else None,
        "average_roll_pitch_stability_confidence_percent": float(np.mean(stability_axes)) if stability_axes else None,
        "average_all_confidence_percent": float(np.mean(all_scores)) if all_scores else None,
        "axis_metrics": rows,
    }


def derivative(t: np.ndarray, y: np.ndarray) -> np.ndarray:
    if t.size < 3 or y.size < 3:
        return np.empty(0, dtype=float)
    return np.gradient(y.reshape(-1)) / np.maximum(np.gradient(t.reshape(-1)), 1e-6)


def metric_annotation(metrics: dict[str, Any]) -> str:
    if int(metrics.get("count", 0) or 0) <= 0:
        return "metrics unavailable"
    corr = metrics.get("correlation")
    ratio = metrics.get("sim_real_rms_ratio")
    corr_text = f"{float(corr):+.3f}" if corr is not None else "n/a"
    ratio_text = f"{float(ratio):.3f}" if ratio is not None else "n/a"
    return (
        f"RMSE {float(metrics.get('rmse', 0.0) or 0.0):.4g} | "
        f"MAE {float(metrics.get('mae', 0.0) or 0.0):.4g} | "
        f"bias(sim-real) {float(metrics.get('bias', 0.0) or 0.0):+.4g}\n"
        f"corr {corr_text} | sim/real RMS {ratio_text}"
        f" | lag {float(metrics.get('lag_s', 0.0) or 0.0):+.2f}s | "
        f"confidence {float(metrics.get('confidence_score', 0.0) or 0.0):.1f}"
    )


def plot_axis_overlay(
    out_path: Path,
    real_t: np.ndarray,
    real_y: np.ndarray,
    sim_t: np.ndarray,
    sim_y: np.ndarray,
    *,
    title: str,
    unit: str,
    metrics: dict[str, Any],
    sim_offset: float = 0.0,
    real_color: str = "black",
    sim_color: str = "red",
) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(12, 4.2))
    real_y = np.asarray(real_y, dtype=float).reshape(-1)
    sim_y = np.asarray(sim_y, dtype=float).reshape(-1)
    if real_t.size and real_y.size:
        ax.plot(real_t, real_y, color=real_color, lw=0.85, label="real")
    if sim_t.size and sim_y.size:
        ax.plot(sim_t, sim_y - float(sim_offset), color=sim_color, lw=0.85, label="sim")
    ax.set_title(title)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(unit)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", fontsize=8)
    ax.text(
        0.01,
        0.98,
        metric_annotation(metrics),
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=8,
        bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "none", "pad": 3},
    )
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_channel_overlay(
    out_path: Path,
    real_t: np.ndarray,
    real_v: np.ndarray,
    sim_t: np.ndarray,
    sim_v: np.ndarray,
    *,
    title: str,
    unit: str,
    real_color: str,
    sim_color: str,
    max_channels: int = 8,
) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(12, 4.8))
    if real_v.size:
        real_v = np.asarray(real_v, dtype=float)
        if real_v.ndim == 1:
            real_v = real_v.reshape((-1, 1))
        for idx in range(min(max_channels, real_v.shape[1])):
            ax.plot(real_t, real_v[:, idx], color=real_color, lw=0.75, alpha=0.28 + 0.07 * (idx % 4), label=f"real ch{idx + 1}")
    if sim_v.size:
        sim_v = np.asarray(sim_v, dtype=float)
        if sim_v.ndim == 1:
            sim_v = sim_v.reshape((-1, 1))
        for idx in range(min(max_channels, sim_v.shape[1])):
            ax.plot(sim_t, sim_v[:, idx], color=sim_color, lw=0.8, alpha=0.30 + 0.07 * (idx % 4), linestyle="--", label=f"sim ch{idx + 1}")
    ax.set_title(title)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(unit)
    ax.grid(True, alpha=0.25)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles[:16], labels[:16], loc="upper right", ncol=4, fontsize=7)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def crop_time_series(
    t: np.ndarray,
    v: np.ndarray,
    start_s: float,
    end_s: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    t = np.asarray(t, dtype=float).reshape(-1)
    if t.size == 0:
        return t, v
    mask = t >= float(start_s)
    if end_s is not None:
        mask &= t <= float(end_s)
    if not np.any(mask):
        return t[:0], np.asarray(v)[:0]
    return t[mask] - float(start_s), np.asarray(v)[mask]


def write_axis_metrics_csv(path: Path, rows: list[tuple[str, dict[str, Any]]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "axis",
                "count",
                "rmse",
                "mae",
                "bias_sim_minus_real",
                "real_rms",
                "sim_rms",
                "sim_real_rms_ratio",
                "correlation",
                "lag_s",
                "confidence_score",
            ]
        )
        for name, metrics in rows:
            writer.writerow(
                [
                    name,
                    int(metrics.get("count", 0) or 0),
                    metrics.get("rmse"),
                    metrics.get("mae"),
                    metrics.get("bias"),
                    metrics.get("real_rms"),
                    metrics.get("sim_rms"),
                    metrics.get("sim_real_rms_ratio"),
                    metrics.get("correlation"),
                    metrics.get("lag_s"),
                    metrics.get("confidence_score"),
                ]
            )


def plot_overlay(
    out_path: Path,
    real_dvl_t: np.ndarray,
    real_dvl: np.ndarray,
    sim_dvl_t: np.ndarray,
    sim_dvl: np.ndarray,
    real_imu_t: np.ndarray,
    real_gyro: np.ndarray,
    sim_imu_t: np.ndarray,
    sim_gyro: np.ndarray,
    real_depth_t: np.ndarray,
    real_depth: np.ndarray,
    sim_depth_t: np.ndarray,
    sim_depth: np.ndarray,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=False)
    if real_dvl.size:
        axes[0].plot(real_dvl_t, real_dvl[:, 0], "k", lw=0.8, label="real DVL x")
        axes[0].plot(real_dvl_t, real_dvl[:, 1], color="0.45", lw=0.8, label="real DVL y")
    if sim_dvl.size:
        axes[0].plot(sim_dvl_t, sim_dvl[:, 0], "tab:red", lw=0.8, label="closed-loop sim DVL x")
        axes[0].plot(sim_dvl_t, sim_dvl[:, 1], "tab:orange", lw=0.8, label="closed-loop sim DVL y")
    axes[0].set_ylabel("DVL m/s")
    axes[0].legend(loc="upper right", ncol=2, fontsize=8)

    if real_gyro.size:
        axes[1].plot(real_imu_t, real_gyro[:, 2], "k", lw=0.8, label="real gyro z")
    if sim_gyro.size:
        axes[1].plot(sim_imu_t, sim_gyro[:, 2], "tab:red", lw=0.8, label="closed-loop sim gyro z")
    axes[1].set_ylabel("yaw rate rad/s")
    axes[1].legend(loc="upper right", fontsize=8)

    if real_depth.size:
        axes[2].plot(real_depth_t, real_depth, "k", lw=0.8, label="real depth")
    if sim_depth.size:
        axes[2].plot(sim_depth_t, sim_depth, "tab:red", lw=0.8, label="closed-loop sim depth")
    axes[2].set_ylabel("depth m")
    axes[2].set_xlabel("bag time s")
    axes[2].legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    plt.savefig(out_path, dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--real-bag", type=Path, default=DEFAULT_REAL_BAG)
    parser.add_argument("--sim-bag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--crop-start-s", type=float, default=5.0)
    parser.add_argument("--crop-end-s", type=float, default=None)
    parser.add_argument(
        "--align-rc-start",
        dest="align_rc_start",
        action="store_true",
        default=True,
        help="Shift sim times so first sim /mavros/rc/override aligns with first real /mavros/rc/override.",
    )
    parser.add_argument(
        "--no-align-rc-start",
        dest="align_rc_start",
        action="store_false",
        help="Use raw bag timestamps without RC-start alignment.",
    )
    parser.add_argument("--align-rc-topic", default="/mavros/rc/override:channels")
    parser.add_argument(
        "--real-start-offset-s",
        type=float,
        default=0.0,
        help="When comparing a cropped replay, align to the first real command at or after this bag offset.",
    )
    args = parser.parse_args()

    real_db = resolve_db3(args.real_bag)
    sim_db = resolve_db3(args.sim_bag)
    real = read_bag(real_db)
    sim = read_bag(sim_db)

    real_dvl_t, real_dvl, real_dvl_key = first_array(real, ["/dvl/twist:linear_m_s"], 3)
    sim_dvl_t, sim_dvl, sim_dvl_key = first_array(
        sim,
        [
            "/dvl/twist:linear_m_s",
            "/dvl/odometry:linear_m_s",
            "/mavros/local_position/velocity_local:linear_m_s",
            "/mavros/local_position/odom:linear_m_s",
        ],
        3,
    )
    real_imu_t, real_gyro, real_gyro_key = first_array(real, ["/mavros/imu/data:gyro_rad_s"], 3)
    sim_imu_t, sim_gyro, sim_gyro_key = first_array(sim, ["/mavros/imu/data:gyro_rad_s"], 3)
    real_att_t, real_att, real_att_key = first_array(
        real,
        [
            "/mavros/imu/data:rpy_rad",
            "/mavros/local_position/pose:rpy_rad",
        ],
        3,
    )
    sim_att_t, sim_att, sim_att_key = first_array(
        sim,
        [
            "/mavros/imu/data:rpy_rad",
            "/mavros/local_position/pose:rpy_rad",
        ],
        3,
    )
    real_depth_t, real_depth, real_depth_key = first_array(real, ["/depth/pose:depth_positive_m"])
    sim_depth_t, sim_depth, sim_depth_key = first_array(
        sim,
        [
            "/depth/pose:depth_positive_m",
            "/depth:depth_positive_m",
        ],
    )
    real_rc_input_t, real_rc_input, real_rc_input_key = first_array(real, ["/mavros/rc/override:channels"])
    sim_rc_input_t, sim_rc_input, sim_rc_input_key = first_array(sim, ["/mavros/rc/override:channels"])
    real_rcin_t, real_rcin, real_rcin_key = first_array(real, ["/mavros/rc/in:channels"])
    sim_rcin_t, sim_rcin, sim_rcin_key = first_array(sim, ["/mavros/rc/in:channels"])
    real_rcou_t, real_rcou, real_rcou_key = first_array(real, ["/mavros/rc/out:channels"])
    sim_rcou_t, sim_rcou, sim_rcou_key = first_array(sim, ["/mavros/rc/out:channels"])
    real_local_odom_t, real_local_odom, real_local_odom_key = first_array(real, ["/mavros/local_position/odom:xyz_m"], 3)
    sim_local_odom_t, sim_local_odom, sim_local_odom_key = first_array(sim, ["/mavros/local_position/odom:xyz_m"], 3)
    real_local_vel_t, real_local_vel, real_local_vel_key = first_array(
        real,
        [
            "/mavros/local_position/velocity_local:linear_m_s",
            "/mavros/local_position/odom:linear_m_s",
        ],
        3,
    )
    sim_local_vel_t, sim_local_vel, sim_local_vel_key = first_array(
        sim,
        [
            "/mavros/local_position/velocity_local:linear_m_s",
            "/mavros/local_position/odom:linear_m_s",
        ],
        3,
    )
    real_static_pressure_t, real_static_pressure, real_static_pressure_key = first_array(real, ["/mavros/imu/static_pressure:pressure_pa"])
    sim_static_pressure_t, sim_static_pressure, sim_static_pressure_key = first_array(sim, ["/mavros/imu/static_pressure:pressure_pa"])
    real_depth = real_depth.reshape(-1) if real_depth.size else real_depth
    sim_depth = sim_depth.reshape(-1) if sim_depth.size else sim_depth

    alignment = {
        "status": "disabled",
        "method": "disabled",
        "topic_key": args.align_rc_topic,
        "real_key": "",
        "sim_key": "",
        "real_start_s": None,
        "real_start_offset_s": float(args.real_start_offset_s),
        "sim_start_s": None,
        "sim_time_shift_s": 0.0,
    }
    if args.align_rc_start:
        alignment = rc_start_alignment(
            real,
            sim,
            real_db,
            sim_db,
            args.align_rc_topic,
            real_start_offset_s=args.real_start_offset_s,
        )
        shift_s = float(alignment.get("sim_time_shift_s", 0.0) or 0.0)
        if alignment.get("status") == "applied" and abs(shift_s) > 1e-12:
            sim_dvl_t = sim_dvl_t + shift_s
            sim_imu_t = sim_imu_t + shift_s
            sim_att_t = sim_att_t + shift_s
            sim_depth_t = sim_depth_t + shift_s
            sim_rc_input_t = sim_rc_input_t + shift_s
            sim_rcin_t = sim_rcin_t + shift_s
            sim_rcou_t = sim_rcou_t + shift_s
            sim_local_odom_t = sim_local_odom_t + shift_s
            sim_local_vel_t = sim_local_vel_t + shift_s
            sim_static_pressure_t = sim_static_pressure_t + shift_s

    depth_rate_real = derivative(real_depth_t, real_depth)
    depth_rate_sim = derivative(sim_depth_t, sim_depth)
    dvl_velocity_metrics = compare_vector(
        real_dvl_t,
        real_dvl,
        sim_dvl_t,
        sim_dvl,
        names=("x", "y", "z"),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
    )
    imu_gyro_metrics = compare_vector(
        real_imu_t,
        real_gyro,
        sim_imu_t,
        sim_gyro,
        names=("x", "y", "z"),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
    )
    attitude_rpy_metrics = compare_vector(
        real_att_t,
        real_att,
        sim_att_t,
        sim_att,
        names=("roll", "pitch", "yaw"),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
    )
    normalized_score = velocity_gyro_score_no_depth(dvl_velocity_metrics, imu_gyro_metrics)
    operational_confidence = operational_confidence_no_depth(
        dvl_velocity_metrics,
        imu_gyro_metrics,
        normalized_score,
    )
    depth_metrics = compare_scalar_with_lag(
        real_depth_t,
        real_depth,
        sim_depth_t,
        sim_depth,
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
        lag_min_s=-2.0,
        lag_max_s=2.0,
        lag_step_s=0.05,
    )
    depth_shape_metrics_offset_removed = compare_scalar_with_lag(
        real_depth_t,
        real_depth,
        sim_depth_t,
        sim_depth,
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
        lag_min_s=-2.0,
        lag_max_s=2.0,
        lag_step_s=0.05,
        remove_median_offset=True,
    )
    depth_rate_metrics = compare_scalar_with_lag(
        real_depth_t,
        depth_rate_real,
        sim_depth_t,
        depth_rate_sim,
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
        lag_min_s=-2.0,
        lag_max_s=2.0,
        lag_step_s=0.05,
    )
    local_position_z_metrics = compare_scalar_with_lag(
        real_local_odom_t,
        real_local_odom[:, 2] if real_local_odom.size else np.empty(0),
        sim_local_odom_t,
        sim_local_odom[:, 2] if sim_local_odom.size else np.empty(0),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
        lag_min_s=-2.0,
        lag_max_s=2.0,
        lag_step_s=0.05,
    )
    local_velocity_z_metrics = compare_scalar_with_lag(
        real_local_vel_t,
        real_local_vel[:, 2] if real_local_vel.size else np.empty(0),
        sim_local_vel_t,
        sim_local_vel[:, 2] if sim_local_vel.size else np.empty(0),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
        lag_min_s=-2.0,
        lag_max_s=2.0,
        lag_step_s=0.05,
    )
    static_pressure_metrics = compare_scalar_with_lag(
        real_static_pressure_t,
        real_static_pressure.reshape(-1) if real_static_pressure.size else np.empty(0),
        sim_static_pressure_t,
        sim_static_pressure.reshape(-1) if sim_static_pressure.size else np.empty(0),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
        lag_min_s=-2.0,
        lag_max_s=2.0,
        lag_step_s=0.05,
        remove_median_offset=True,
    )
    rc_input_channel_metrics = compare_matrix_columns(
        real_rc_input_t,
        real_rc_input,
        sim_rc_input_t,
        sim_rc_input,
        names=tuple(f"ch{idx}" for idx in range(1, 9)),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
    )
    rcin_channel_metrics = compare_matrix_columns(
        real_rcin_t,
        real_rcin,
        sim_rcin_t,
        sim_rcin,
        names=tuple(f"ch{idx}" for idx in range(1, 9)),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
    )
    rc_override_to_sim_rcin_metrics = compare_matrix_columns(
        real_rc_input_t,
        real_rc_input,
        sim_rcin_t,
        sim_rcin,
        names=tuple(f"ch{idx}" for idx in range(1, 9)),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
    )
    rcout_channel_metrics = compare_matrix_columns(
        real_rcou_t,
        real_rcou,
        sim_rcou_t,
        sim_rcou,
        names=tuple(f"ch{idx}" for idx in range(1, 9)),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
    )
    real_rcou_vertical_axes = rcout_vertical_axes(real_rcou)
    sim_rcou_vertical_axes = rcout_vertical_axes(sim_rcou)
    rcout_vertical_axis_metrics = compare_matrix_columns(
        real_rcou_t,
        real_rcou_vertical_axes,
        sim_rcou_t,
        sim_rcou_vertical_axes,
        names=("vertical_common", "vertical_roll_diff", "vertical_pitch_diff"),
        crop_start_s=args.crop_start_s,
        crop_end_s=args.crop_end_s,
    )

    plots_dir = args.output_dir / "axis_plots"
    plot_files = {
        "dvl_vx_surge": str(plots_dir / "dvl_vx_surge.png"),
        "dvl_vy_sway": str(plots_dir / "dvl_vy_sway.png"),
        "dvl_vz_heave": str(plots_dir / "dvl_vz_heave.png"),
        "gyro_x_roll_rate": str(plots_dir / "gyro_x_roll_rate.png"),
        "gyro_y_pitch_rate": str(plots_dir / "gyro_y_pitch_rate.png"),
        "gyro_z_yaw_rate": str(plots_dir / "gyro_z_yaw_rate.png"),
        "attitude_roll": str(plots_dir / "attitude_roll.png"),
        "attitude_pitch": str(plots_dir / "attitude_pitch.png"),
        "attitude_yaw": str(plots_dir / "attitude_yaw.png"),
        "depth_raw": str(plots_dir / "depth_raw.png"),
        "depth_offset_removed": str(plots_dir / "depth_offset_removed.png"),
        "depth_rate": str(plots_dir / "depth_rate.png"),
        "rc_input_channels": str(plots_dir / "rc_input_channels.png"),
        "rcin_channels": str(plots_dir / "rcin_channels.png"),
        "rc_override_to_sim_rcin_channels": str(plots_dir / "rc_override_to_sim_rcin_channels.png"),
        "rcou_output_channels": str(plots_dir / "rcou_output_channels.png"),
        "local_position_z": str(plots_dir / "local_position_z.png"),
        "local_velocity_z": str(plots_dir / "local_velocity_z.png"),
        "static_pressure_offset_removed": str(plots_dir / "static_pressure_offset_removed.png"),
        "rcou_vertical_common": str(plots_dir / "rcou_vertical_common.png"),
        "rcou_vertical_roll_diff": str(plots_dir / "rcou_vertical_roll_diff.png"),
        "rcou_vertical_pitch_diff": str(plots_dir / "rcou_vertical_pitch_diff.png"),
        "axis_metrics_csv": str(args.output_dir / "axis_metrics.csv"),
    }

    payload = {
        "real_bag": str(real_db),
        "sim_bag": str(sim_db),
        "crop_start_s": float(args.crop_start_s),
        "crop_end_s": None if args.crop_end_s is None else float(args.crop_end_s),
        "time_alignment": alignment,
        "keys": {
            "real_dvl": real_dvl_key,
            "sim_dvl": sim_dvl_key,
            "real_gyro": real_gyro_key,
            "sim_gyro": sim_gyro_key,
            "real_attitude": real_att_key,
            "sim_attitude": sim_att_key,
            "real_depth": real_depth_key,
            "sim_depth": sim_depth_key,
            "real_rc_input": real_rc_input_key,
            "sim_rc_input": sim_rc_input_key,
            "real_rcin": real_rcin_key,
            "sim_rcin": sim_rcin_key,
            "real_rcou": real_rcou_key,
            "sim_rcou": sim_rcou_key,
            "real_local_odom": real_local_odom_key,
            "sim_local_odom": sim_local_odom_key,
            "real_local_velocity": real_local_vel_key,
            "sim_local_velocity": sim_local_vel_key,
            "real_static_pressure": real_static_pressure_key,
            "sim_static_pressure": sim_static_pressure_key,
        },
        "counts": {
            "real_dvl": int(real_dvl_t.size),
            "sim_dvl": int(sim_dvl_t.size),
            "real_imu": int(real_imu_t.size),
            "sim_imu": int(sim_imu_t.size),
            "real_attitude": int(real_att_t.size),
            "sim_attitude": int(sim_att_t.size),
            "real_depth": int(real_depth_t.size),
            "sim_depth": int(sim_depth_t.size),
            "real_rc_input": int(real_rc_input_t.size),
            "sim_rc_input": int(sim_rc_input_t.size),
            "real_rcin": int(real_rcin_t.size),
            "sim_rcin": int(sim_rcin_t.size),
            "real_rcou": int(real_rcou_t.size),
            "sim_rcou": int(sim_rcou_t.size),
            "real_local_odom": int(real_local_odom_t.size),
            "sim_local_odom": int(sim_local_odom_t.size),
            "real_local_velocity": int(real_local_vel_t.size),
            "sim_local_velocity": int(sim_local_vel_t.size),
            "real_static_pressure": int(real_static_pressure_t.size),
            "sim_static_pressure": int(sim_static_pressure_t.size),
        },
        "dvl_velocity_metrics": dvl_velocity_metrics,
        "imu_gyro_metrics": imu_gyro_metrics,
        "attitude_rpy_metrics": attitude_rpy_metrics,
        "velocity_gyro_score_no_depth": normalized_score,
        "operational_confidence_no_depth": operational_confidence,
        "depth_metrics": depth_metrics,
        "depth_shape_metrics_offset_removed": depth_shape_metrics_offset_removed,
        "depth_rate_metrics": depth_rate_metrics,
        "local_position_z_metrics": local_position_z_metrics,
        "local_velocity_z_metrics": local_velocity_z_metrics,
        "static_pressure_metrics_offset_removed": static_pressure_metrics,
        "rc_input_channel_metrics": rc_input_channel_metrics,
        "rcin_channel_metrics": rcin_channel_metrics,
        "rc_override_to_sim_rcin_channel_metrics": rc_override_to_sim_rcin_metrics,
        "rcout_channel_metrics": rcout_channel_metrics,
        "rcout_vertical_axis_metrics": rcout_vertical_axis_metrics,
        "plot_files": plot_files,
        "distributions": {
            "real_dvl": vector_stats(real_dvl_t, real_dvl, ("x", "y", "z")) if real_dvl.size else {},
            "sim_dvl": vector_stats(sim_dvl_t, sim_dvl, ("x", "y", "z")) if sim_dvl.size else {},
            "real_gyro": vector_stats(real_imu_t, real_gyro, ("x", "y", "z")) if real_gyro.size else {},
            "sim_gyro": vector_stats(sim_imu_t, sim_gyro, ("x", "y", "z")) if sim_gyro.size else {},
            "real_attitude_rpy": vector_stats(real_att_t, real_att, ("roll", "pitch", "yaw")) if real_att.size else {},
            "sim_attitude_rpy": vector_stats(sim_att_t, sim_att, ("roll", "pitch", "yaw")) if sim_att.size else {},
        },
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "closed_loop_real_vs_sim_summary.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False)
    )
    axis_rows = [
        ("dvl_vx_surge", dvl_velocity_metrics.get("x", {})),
        ("dvl_vy_sway", dvl_velocity_metrics.get("y", {})),
        ("dvl_vz_heave", dvl_velocity_metrics.get("z", {})),
        ("gyro_x_roll_rate", imu_gyro_metrics.get("x", {})),
        ("gyro_y_pitch_rate", imu_gyro_metrics.get("y", {})),
        ("gyro_z_yaw_rate", imu_gyro_metrics.get("z", {})),
        ("attitude_roll", attitude_rpy_metrics.get("roll", {})),
        ("attitude_pitch", attitude_rpy_metrics.get("pitch", {})),
        ("attitude_yaw", attitude_rpy_metrics.get("yaw", {})),
        ("depth_raw", depth_metrics),
        ("depth_offset_removed", depth_shape_metrics_offset_removed),
        ("depth_rate", depth_rate_metrics),
        ("local_position_z", local_position_z_metrics),
        ("local_velocity_z", local_velocity_z_metrics),
        ("static_pressure_offset_removed", static_pressure_metrics),
    ]
    def matrix_metric_rows(prefix: str, metrics_by_name: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
        if not isinstance(metrics_by_name, dict):
            return []
        if any(isinstance(value, dict) for value in metrics_by_name.values()):
            return [
                (f"{prefix}_{name}", metrics)
                for name, metrics in metrics_by_name.items()
                if isinstance(metrics, dict)
            ]
        return [(prefix, metrics_by_name)]

    axis_rows.extend(matrix_metric_rows("rc_input", rc_input_channel_metrics))
    axis_rows.extend(matrix_metric_rows("rcin", rcin_channel_metrics))
    axis_rows.extend(matrix_metric_rows("rc_override_to_sim_rcin", rc_override_to_sim_rcin_metrics))
    axis_rows.extend(matrix_metric_rows("rcout", rcout_channel_metrics))
    axis_rows.extend(matrix_metric_rows("rcout", rcout_vertical_axis_metrics))
    write_axis_metrics_csv(args.output_dir / "axis_metrics.csv", axis_rows)
    real_dvl_plot_t, real_dvl_plot = crop_time_series(real_dvl_t, real_dvl, args.crop_start_s, args.crop_end_s)
    sim_dvl_plot_t, sim_dvl_plot = crop_time_series(sim_dvl_t, sim_dvl, args.crop_start_s, args.crop_end_s)
    real_imu_plot_t, real_gyro_plot = crop_time_series(real_imu_t, real_gyro, args.crop_start_s, args.crop_end_s)
    sim_imu_plot_t, sim_gyro_plot = crop_time_series(sim_imu_t, sim_gyro, args.crop_start_s, args.crop_end_s)
    real_att_plot_t, real_att_plot = crop_time_series(real_att_t, real_att, args.crop_start_s, args.crop_end_s)
    sim_att_plot_t, sim_att_plot = crop_time_series(sim_att_t, sim_att, args.crop_start_s, args.crop_end_s)
    real_depth_plot_t, real_depth_plot = crop_time_series(real_depth_t, real_depth, args.crop_start_s, args.crop_end_s)
    sim_depth_plot_t, sim_depth_plot = crop_time_series(sim_depth_t, sim_depth, args.crop_start_s, args.crop_end_s)
    real_rc_input_plot_t, real_rc_input_plot = crop_time_series(real_rc_input_t, real_rc_input, args.crop_start_s, args.crop_end_s)
    sim_rc_input_plot_t, sim_rc_input_plot = crop_time_series(sim_rc_input_t, sim_rc_input, args.crop_start_s, args.crop_end_s)
    real_rcin_plot_t, real_rcin_plot = crop_time_series(real_rcin_t, real_rcin, args.crop_start_s, args.crop_end_s)
    sim_rcin_plot_t, sim_rcin_plot = crop_time_series(sim_rcin_t, sim_rcin, args.crop_start_s, args.crop_end_s)
    real_rcou_plot_t, real_rcou_plot = crop_time_series(real_rcou_t, real_rcou, args.crop_start_s, args.crop_end_s)
    sim_rcou_plot_t, sim_rcou_plot = crop_time_series(sim_rcou_t, sim_rcou, args.crop_start_s, args.crop_end_s)
    real_local_odom_plot_t, real_local_odom_plot = crop_time_series(real_local_odom_t, real_local_odom, args.crop_start_s, args.crop_end_s)
    sim_local_odom_plot_t, sim_local_odom_plot = crop_time_series(sim_local_odom_t, sim_local_odom, args.crop_start_s, args.crop_end_s)
    real_local_vel_plot_t, real_local_vel_plot = crop_time_series(real_local_vel_t, real_local_vel, args.crop_start_s, args.crop_end_s)
    sim_local_vel_plot_t, sim_local_vel_plot = crop_time_series(sim_local_vel_t, sim_local_vel, args.crop_start_s, args.crop_end_s)
    real_static_pressure_plot_t, real_static_pressure_plot = crop_time_series(
        real_static_pressure_t,
        real_static_pressure.reshape(-1) if real_static_pressure.size else real_static_pressure,
        args.crop_start_s,
        args.crop_end_s,
    )
    sim_static_pressure_plot_t, sim_static_pressure_plot = crop_time_series(
        sim_static_pressure_t,
        sim_static_pressure.reshape(-1) if sim_static_pressure.size else sim_static_pressure,
        args.crop_start_s,
        args.crop_end_s,
    )
    real_rcou_vertical_plot = rcout_vertical_axes(real_rcou_plot)
    sim_rcou_vertical_plot = rcout_vertical_axes(sim_rcou_plot)
    depth_rate_real_plot = derivative(real_depth_plot_t, real_depth_plot)
    depth_rate_sim_plot = derivative(sim_depth_plot_t, sim_depth_plot)
    plot_axis_overlay(
        plots_dir / "dvl_vx_surge.png",
        real_dvl_plot_t,
        real_dvl_plot[:, 0] if real_dvl_plot.size else np.empty(0),
        sim_dvl_plot_t,
        sim_dvl_plot[:, 0] if sim_dvl_plot.size else np.empty(0),
        title="DVL vx / surge",
        unit="m/s",
        metrics=dvl_velocity_metrics.get("x", {}),
    )
    plot_axis_overlay(
        plots_dir / "dvl_vy_sway.png",
        real_dvl_plot_t,
        real_dvl_plot[:, 1] if real_dvl_plot.size else np.empty(0),
        sim_dvl_plot_t,
        sim_dvl_plot[:, 1] if sim_dvl_plot.size else np.empty(0),
        title="DVL vy / sway",
        unit="m/s",
        metrics=dvl_velocity_metrics.get("y", {}),
    )
    plot_axis_overlay(
        plots_dir / "dvl_vz_heave.png",
        real_dvl_plot_t,
        real_dvl_plot[:, 2] if real_dvl_plot.size else np.empty(0),
        sim_dvl_plot_t,
        sim_dvl_plot[:, 2] if sim_dvl_plot.size else np.empty(0),
        title="DVL vz / heave",
        unit="m/s",
        metrics=dvl_velocity_metrics.get("z", {}),
    )
    plot_axis_overlay(
        plots_dir / "gyro_x_roll_rate.png",
        real_imu_plot_t,
        real_gyro_plot[:, 0] if real_gyro_plot.size else np.empty(0),
        sim_imu_plot_t,
        sim_gyro_plot[:, 0] if sim_gyro_plot.size else np.empty(0),
        title="gyro x / roll rate",
        unit="rad/s",
        metrics=imu_gyro_metrics.get("x", {}),
    )
    plot_axis_overlay(
        plots_dir / "gyro_y_pitch_rate.png",
        real_imu_plot_t,
        real_gyro_plot[:, 1] if real_gyro_plot.size else np.empty(0),
        sim_imu_plot_t,
        sim_gyro_plot[:, 1] if sim_gyro_plot.size else np.empty(0),
        title="gyro y / pitch rate",
        unit="rad/s",
        metrics=imu_gyro_metrics.get("y", {}),
    )
    plot_axis_overlay(
        plots_dir / "gyro_z_yaw_rate.png",
        real_imu_plot_t,
        real_gyro_plot[:, 2] if real_gyro_plot.size else np.empty(0),
        sim_imu_plot_t,
        sim_gyro_plot[:, 2] if sim_gyro_plot.size else np.empty(0),
        title="gyro z / yaw rate",
        unit="rad/s",
        metrics=imu_gyro_metrics.get("z", {}),
    )
    plot_axis_overlay(
        plots_dir / "attitude_roll.png",
        real_att_plot_t,
        real_att_plot[:, 0] if real_att_plot.size else np.empty(0),
        sim_att_plot_t,
        sim_att_plot[:, 0] if sim_att_plot.size else np.empty(0),
        title="FCU attitude roll",
        unit="rad",
        metrics=attitude_rpy_metrics.get("roll", {}),
    )
    plot_axis_overlay(
        plots_dir / "attitude_pitch.png",
        real_att_plot_t,
        real_att_plot[:, 1] if real_att_plot.size else np.empty(0),
        sim_att_plot_t,
        sim_att_plot[:, 1] if sim_att_plot.size else np.empty(0),
        title="FCU attitude pitch",
        unit="rad",
        metrics=attitude_rpy_metrics.get("pitch", {}),
    )
    plot_axis_overlay(
        plots_dir / "attitude_yaw.png",
        real_att_plot_t,
        real_att_plot[:, 2] if real_att_plot.size else np.empty(0),
        sim_att_plot_t,
        sim_att_plot[:, 2] if sim_att_plot.size else np.empty(0),
        title="FCU attitude yaw",
        unit="rad",
        metrics=attitude_rpy_metrics.get("yaw", {}),
    )
    plot_axis_overlay(
        plots_dir / "depth_raw.png",
        real_depth_plot_t,
        real_depth_plot,
        sim_depth_plot_t,
        sim_depth_plot,
        title="depth raw",
        unit="m",
        metrics=depth_metrics,
    )
    plot_axis_overlay(
        plots_dir / "depth_offset_removed.png",
        real_depth_plot_t,
        real_depth_plot,
        sim_depth_plot_t,
        sim_depth_plot,
        title="depth offset removed",
        unit="m",
        metrics=depth_shape_metrics_offset_removed,
        sim_offset=float(depth_shape_metrics_offset_removed.get("removed_median_offset", 0.0) or 0.0),
    )
    plot_axis_overlay(
        plots_dir / "depth_rate.png",
        real_depth_plot_t,
        depth_rate_real_plot,
        sim_depth_plot_t,
        depth_rate_sim_plot,
        title="depth rate",
        unit="m/s",
        metrics=depth_rate_metrics,
    )
    plot_channel_overlay(
        plots_dir / "rc_input_channels.png",
        real_rc_input_plot_t,
        real_rc_input_plot,
        sim_rc_input_plot_t,
        sim_rc_input_plot,
        title="RC input channels",
        unit="PWM us",
        real_color="0.25",
        sim_color="tab:blue",
    )
    plot_channel_overlay(
        plots_dir / "rcin_channels.png",
        real_rcin_plot_t,
        real_rcin_plot,
        sim_rcin_plot_t,
        sim_rcin_plot,
        title="FCU RC input channels",
        unit="PWM us",
        real_color="0.25",
        sim_color="tab:blue",
    )
    plot_channel_overlay(
        plots_dir / "rc_override_to_sim_rcin_channels.png",
        real_rc_input_plot_t,
        real_rc_input_plot,
        sim_rcin_plot_t,
        sim_rcin_plot,
        title="real RC override vs sim FCU RC input",
        unit="PWM us",
        real_color="0.25",
        sim_color="tab:blue",
    )
    plot_channel_overlay(
        plots_dir / "rcou_output_channels.png",
        real_rcou_plot_t,
        real_rcou_plot,
        sim_rcou_plot_t,
        sim_rcou_plot,
        title="RCOU output channels",
        unit="PWM us",
        real_color="tab:orange",
        sim_color="purple",
    )
    plot_axis_overlay(
        plots_dir / "local_position_z.png",
        real_local_odom_plot_t,
        real_local_odom_plot[:, 2] if real_local_odom_plot.size else np.empty(0),
        sim_local_odom_plot_t,
        sim_local_odom_plot[:, 2] if sim_local_odom_plot.size else np.empty(0),
        title="FCU local position z",
        unit="m",
        metrics=local_position_z_metrics,
    )
    plot_axis_overlay(
        plots_dir / "local_velocity_z.png",
        real_local_vel_plot_t,
        real_local_vel_plot[:, 2] if real_local_vel_plot.size else np.empty(0),
        sim_local_vel_plot_t,
        sim_local_vel_plot[:, 2] if sim_local_vel_plot.size else np.empty(0),
        title="FCU local velocity z",
        unit="m/s",
        metrics=local_velocity_z_metrics,
    )
    plot_axis_overlay(
        plots_dir / "static_pressure_offset_removed.png",
        real_static_pressure_plot_t,
        real_static_pressure_plot,
        sim_static_pressure_plot_t,
        sim_static_pressure_plot,
        title="static pressure offset removed",
        unit="Pa",
        metrics=static_pressure_metrics,
        sim_offset=float(static_pressure_metrics.get("removed_median_offset", 0.0) or 0.0),
    )
    for idx, (key, title) in enumerate(
        (
            ("vertical_common", "RCOU vertical common"),
            ("vertical_roll_diff", "RCOU vertical roll diff"),
            ("vertical_pitch_diff", "RCOU vertical pitch diff"),
        )
    ):
        plot_axis_overlay(
            plots_dir / f"rcou_{key}.png",
            real_rcou_plot_t,
            real_rcou_vertical_plot[:, idx] if real_rcou_vertical_plot.size else np.empty(0),
            sim_rcou_plot_t,
            sim_rcou_vertical_plot[:, idx] if sim_rcou_vertical_plot.size else np.empty(0),
            title=title,
            unit="norm",
            metrics=rcout_vertical_axis_metrics.get(key, {}),
            real_color="tab:orange",
            sim_color="purple",
        )
    plot_overlay(
        args.output_dir / "closed_loop_real_vs_sim_overlay.png",
        real_dvl_plot_t,
        real_dvl_plot,
        sim_dvl_plot_t,
        sim_dvl_plot,
        real_imu_plot_t,
        real_gyro_plot,
        sim_imu_plot_t,
        sim_gyro_plot,
        real_depth_plot_t,
        real_depth_plot,
        sim_depth_plot_t,
        sim_depth_plot,
    )
    if alignment.get("status") == "applied":
        print(
            "[closed-loop-compare] RC-start alignment: "
            f"real={alignment['real_start_s']:.3f}s sim={alignment['sim_start_s']:.3f}s "
            f"shift={alignment['sim_time_shift_s']:+.3f}s"
        )
    else:
        print(f"[closed-loop-compare] RC-start alignment: {alignment.get('status')}")
    print(f"[closed-loop-compare] wrote {args.output_dir}")


if __name__ == "__main__":
    main()

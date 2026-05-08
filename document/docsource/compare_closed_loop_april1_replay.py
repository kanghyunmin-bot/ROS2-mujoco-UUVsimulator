from __future__ import annotations

import argparse
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
DEFAULT_OUT = Path("document/docsource/closed_loop_april1_rc_replay_compare")


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
        t, v = data.array(key, dims)
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
    sim_phase_start = first_string_time(sim_db, "/measurement/phase", "closed_loop_replay")
    if sim_phase_start is not None:
        sim_t = np.asarray([float(sim_phase_start)], dtype=float)
        sim_key = "/measurement/phase:closed_loop_replay:first"
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
        if np.sum(mask) < 12:
            continue
        r = real_y[mask]
        s = np.interp(query_t[mask], sim_t, sim_y)
        finite = np.isfinite(r) & np.isfinite(s)
        r = r[finite]
        s = s[finite]
        if r.size < 12:
            continue
        residual = r - s
        offset = 0.0
        if remove_median_offset:
            offset = float(np.median(residual))
            residual = residual - offset
        corr = None
        if np.std(r) > 1e-12 and np.std(s) > 1e-12:
            corr = float(np.corrcoef(r, s)[0, 1])
        candidate = {
            "count": int(r.size),
            "lag_s": float(lag),
            "rmse": float(np.sqrt(np.mean(residual * residual))),
            "mae": float(np.mean(np.abs(residual))),
            "bias": float(np.mean(residual)),
            "residual_std": float(np.std(residual)),
            "correlation": corr,
            "removed_median_offset": offset,
            "real": scalar_stats(real_t[mask][finite], r),
            "sim": scalar_stats(real_t[mask][finite], s),
            "gain_fit_real_from_sim": fit_gain_offset(s, r),
        }
        if best is None:
            best = candidate
            continue
        best_corr = -1.0 if best["correlation"] is None else abs(float(best["correlation"]))
        cand_corr = -1.0 if candidate["correlation"] is None else abs(float(candidate["correlation"]))
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
            lag_min_s=-2.0,
            lag_max_s=2.0,
            lag_step_s=0.05,
        )
        for idx, name in enumerate(names)
    }


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
    real_depth_t, real_depth, real_depth_key = first_array(real, ["/depth/pose:depth_positive_m"])
    sim_depth_t, sim_depth, sim_depth_key = first_array(
        sim,
        [
            "/depth/pose:depth_positive_m",
            "/depth:depth_positive_m",
        ],
    )
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
            sim_depth_t = sim_depth_t + shift_s

    depth_rate_real = derivative(real_depth_t, real_depth)
    depth_rate_sim = derivative(sim_depth_t, sim_depth)
    dvl_velocity_metrics = compare_vector(
        real_dvl_t,
        real_dvl,
        sim_dvl_t,
        sim_dvl,
        names=("x", "y", "z"),
        crop_start_s=args.crop_start_s,
    )
    imu_gyro_metrics = compare_vector(
        real_imu_t,
        real_gyro,
        sim_imu_t,
        sim_gyro,
        names=("x", "y", "z"),
        crop_start_s=args.crop_start_s,
    )
    normalized_score = velocity_gyro_score_no_depth(dvl_velocity_metrics, imu_gyro_metrics)
    operational_confidence = operational_confidence_no_depth(
        dvl_velocity_metrics,
        imu_gyro_metrics,
        normalized_score,
    )

    payload = {
        "real_bag": str(real_db),
        "sim_bag": str(sim_db),
        "time_alignment": alignment,
        "keys": {
            "real_dvl": real_dvl_key,
            "sim_dvl": sim_dvl_key,
            "real_gyro": real_gyro_key,
            "sim_gyro": sim_gyro_key,
            "real_depth": real_depth_key,
            "sim_depth": sim_depth_key,
        },
        "counts": {
            "real_dvl": int(real_dvl_t.size),
            "sim_dvl": int(sim_dvl_t.size),
            "real_imu": int(real_imu_t.size),
            "sim_imu": int(sim_imu_t.size),
            "real_depth": int(real_depth_t.size),
            "sim_depth": int(sim_depth_t.size),
        },
        "dvl_velocity_metrics": dvl_velocity_metrics,
        "imu_gyro_metrics": imu_gyro_metrics,
        "velocity_gyro_score_no_depth": normalized_score,
        "operational_confidence_no_depth": operational_confidence,
        "depth_metrics": compare_scalar_with_lag(
            real_depth_t,
            real_depth,
            sim_depth_t,
            sim_depth,
            crop_start_s=args.crop_start_s,
            lag_min_s=-2.0,
            lag_max_s=2.0,
            lag_step_s=0.05,
        ),
        "depth_shape_metrics_offset_removed": compare_scalar_with_lag(
            real_depth_t,
            real_depth,
            sim_depth_t,
            sim_depth,
            crop_start_s=args.crop_start_s,
            lag_min_s=-2.0,
            lag_max_s=2.0,
            lag_step_s=0.05,
            remove_median_offset=True,
        ),
        "depth_rate_metrics": compare_scalar_with_lag(
            real_depth_t,
            depth_rate_real,
            sim_depth_t,
            depth_rate_sim,
            crop_start_s=args.crop_start_s,
            lag_min_s=-2.0,
            lag_max_s=2.0,
            lag_step_s=0.05,
        ),
        "distributions": {
            "real_dvl": vector_stats(real_dvl_t, real_dvl, ("x", "y", "z")) if real_dvl.size else {},
            "sim_dvl": vector_stats(sim_dvl_t, sim_dvl, ("x", "y", "z")) if sim_dvl.size else {},
            "real_gyro": vector_stats(real_imu_t, real_gyro, ("x", "y", "z")) if real_gyro.size else {},
            "sim_gyro": vector_stats(sim_imu_t, sim_gyro, ("x", "y", "z")) if sim_gyro.size else {},
        },
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "closed_loop_real_vs_sim_summary.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False)
    )
    plot_overlay(
        args.output_dir / "closed_loop_real_vs_sim_overlay.png",
        real_dvl_t,
        real_dvl,
        sim_dvl_t,
        sim_dvl,
        real_imu_t,
        real_gyro,
        sim_imu_t,
        sim_gyro,
        real_depth_t,
        real_depth,
        sim_depth_t,
        sim_depth,
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

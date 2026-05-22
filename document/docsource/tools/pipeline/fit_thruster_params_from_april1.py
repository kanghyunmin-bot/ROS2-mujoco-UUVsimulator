from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
UUV_DIR = REPO_ROOT / "uuv_mujoco" / "v2.2"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(UUV_DIR) not in sys.path:
    sys.path.insert(0, str(UUV_DIR))

from analyze_april1_real_bags import read_bag, scalar_stats  # noqa: E402
from physics.thruster_mapping import (  # noqa: E402
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
)


DEFAULT_ROOT = Path("real_robot_ros_bag/extracted_2026_04_01")
DEFAULT_OUT = Path("document/docsource/runs/rosbag/real_bag_2026_04_01_thruster_fit")
PWM_CENTER = 1500.0
RC_OVERRIDE_SPAN = 300.0
RC_OUT_SPAN = 400.0


def discover_bags(root: Path) -> list[Path]:
    if root.is_file() and root.suffix == ".db3":
        return [root]
    bags: list[Path] = []
    for db_path in sorted(root.glob("bag_*/**/*.db3")):
        try:
            data = read_bag(db_path)
        except Exception:
            continue
        if data.topic_counts.get("/mavros/rc/override", 0) or data.topic_counts.get("/mavros/rc/out", 0):
            bags.append(db_path)
    return bags


def normalize_rc_override(rc: np.ndarray) -> np.ndarray:
    channels = np.asarray(rc[:, :8], dtype=float)
    return np.clip((channels - PWM_CENTER) / RC_OVERRIDE_SPAN, -1.0, 1.0)


def normalize_rc_out(rc: np.ndarray) -> np.ndarray:
    channels = np.asarray(rc[:, :8], dtype=float)
    valid = (channels >= 800.0) & (channels <= 2200.0)
    out = np.zeros_like(channels, dtype=float)
    out[valid] = (channels[valid] - PWM_CENTER) / RC_OUT_SPAN
    return np.clip(out, -1.0, 1.0)


def resample_matrix(src_t: np.ndarray, src_v: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    if src_t.size == 0 or src_v.size == 0 or dst_t.size == 0:
        cols = src_v.shape[1] if src_v.ndim == 2 else 1
        return np.empty((0, cols), dtype=float)
    values = np.asarray(src_v, dtype=float)
    if values.ndim == 1:
        values = values.reshape((-1, 1))
    return np.column_stack([np.interp(dst_t, src_t, values[:, idx]) for idx in range(values.shape[1])])


def fit_gain_offset(x: np.ndarray, y: np.ndarray) -> dict[str, Any]:
    x = np.asarray(x, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    finite = np.isfinite(x) & np.isfinite(y)
    x = x[finite]
    y = y[finite]
    if x.size < 12 or np.std(x) < 1e-9:
        return {"count": int(x.size), "gain": None, "offset": None, "r2": None}
    A = np.column_stack([np.ones_like(x), x])
    beta, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ beta
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    corr = None
    if np.std(y) > 1e-12 and np.std(pred) > 1e-12:
        corr = float(np.corrcoef(y, pred)[0, 1])
    return {
        "count": int(x.size),
        "offset": float(beta[0]),
        "gain": float(beta[1]),
        "r2": float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else None,
        "correlation": corr,
        "residual_std": float(np.std(y - pred)),
        "x_rms": float(np.sqrt(np.mean(x * x))),
        "y_rms": float(np.sqrt(np.mean(y * y))),
    }


def best_lag_fit(
    cmd_t: np.ndarray,
    cmd: np.ndarray,
    y_t: np.ndarray,
    y: np.ndarray,
    *,
    threshold: float,
    lag_min_s: float,
    lag_max_s: float,
    lag_step_s: float,
) -> dict[str, Any]:
    if cmd_t.size < 12 or y_t.size < 12:
        return {"count": 0}
    best: dict[str, Any] | None = None
    for lag in np.arange(lag_min_s, lag_max_s + 1e-9, lag_step_s):
        query_t = y_t - float(lag)
        mask = (query_t >= cmd_t[0]) & (query_t <= cmd_t[-1]) & np.isfinite(y)
        if np.sum(mask) < 12:
            continue
        x = np.interp(query_t[mask], cmd_t, cmd)
        response = y[mask]
        active = np.abs(x) >= threshold
        if np.sum(active) < 12:
            continue
        fit = fit_gain_offset(x[active], response[active])
        if fit.get("gain") is None:
            continue
        fit["lag_s"] = float(lag)
        fit["active_fraction"] = float(np.mean(active))
        score = abs(float(fit.get("correlation") or 0.0))
        if best is None or score > abs(float(best.get("correlation") or 0.0)):
            best = fit
    return best or {"count": 0}


def signed_fit(
    cmd_t: np.ndarray,
    cmd: np.ndarray,
    y_t: np.ndarray,
    y: np.ndarray,
    *,
    lag_s: float,
    threshold: float,
) -> dict[str, Any]:
    if cmd_t.size < 12 or y_t.size < 12:
        return {"positive": {"count": 0}, "negative": {"count": 0}}
    query_t = y_t - float(lag_s)
    mask = (query_t >= cmd_t[0]) & (query_t <= cmd_t[-1]) & np.isfinite(y)
    if np.sum(mask) < 12:
        return {"positive": {"count": 0}, "negative": {"count": 0}}
    x = np.interp(query_t[mask], cmd_t, cmd)
    response = y[mask]
    pos = x >= threshold
    neg = x <= -threshold
    return {
        "positive": fit_gain_offset(x[pos], response[pos]) if np.sum(pos) >= 12 else {"count": int(np.sum(pos))},
        "negative": fit_gain_offset(x[neg], response[neg]) if np.sum(neg) >= 12 else {"count": int(np.sum(neg))},
    }


def rc_out_thruster_commands(rc_out_norm8: np.ndarray) -> dict[str, np.ndarray]:
    out: dict[str, list[float]] = {name: [] for name in ARDUSUB_VECTORED_6DOF_SERVO_MAP}
    for row in rc_out_norm8:
        values = {name: 0.0 for name in out}
        for idx, name in enumerate(ARDUSUB_VECTORED_6DOF_SERVO_MAP):
            if idx < row.size:
                values[name] = float(np.clip(row[idx] * ARDUSUB_VECTORED_6DOF_SERVO_SIGNS[idx], -1.0, 1.0))
        for name, value in values.items():
            out[name].append(value)
    return {name: np.asarray(values, dtype=float) for name, values in out.items()}


def analyze_bag(db_path: Path, *, threshold: float, lag_min_s: float, lag_max_s: float, lag_step_s: float) -> dict[str, Any]:
    data = read_bag(db_path)
    rc_override_t, rc_override = data.array("/mavros/rc/override:channels")
    rc_out_t, rc_out = data.array("/mavros/rc/out:channels")
    dvl_t, dvl_vel = data.array("/dvl/twist:linear_m_s", 3)
    imu_t, gyro = data.array("/mavros/imu/data:gyro_rad_s", 3)
    depth_t, depth = data.array("/depth/pose:depth_positive_m")

    out: dict[str, Any] = {
        "db_path": str(db_path),
        "bag_name": data.name,
        "duration_s": float(data.duration_s),
        "counts": {
            "rc_override": int(rc_override_t.size),
            "rc_out": int(rc_out_t.size),
            "dvl": int(dvl_t.size),
            "imu": int(imu_t.size),
            "depth": int(depth_t.size),
        },
        "notes": [
            "This is a coarse command-response fit, not a direct bollard-pull force fit.",
            "Use it to choose which signed thruster gains deserve replay testing.",
        ],
    }

    if rc_override.size and (dvl_vel.size or gyro.size or depth.size):
        rc_norm = normalize_rc_override(rc_override)
        axes = {
            "surge_from_rc_override_ch5": (rc_override_t, rc_norm[:, 4], dvl_t, dvl_vel[:, 0] if dvl_vel.size else np.empty(0)),
            "sway_from_rc_override_ch6": (rc_override_t, rc_norm[:, 5], dvl_t, dvl_vel[:, 1] if dvl_vel.size else np.empty(0)),
            "heave_from_rc_override_minus_ch3": (
                rc_override_t,
                -rc_norm[:, 2],
                dvl_t,
                dvl_vel[:, 2] if dvl_vel.size else np.empty(0),
            ),
            "yaw_rate_from_rc_override_minus_ch4": (
                rc_override_t,
                -rc_norm[:, 3],
                imu_t,
                gyro[:, 2] if gyro.size else np.empty(0),
            ),
        }
        fits: dict[str, Any] = {}
        for name, (cmd_t, cmd, y_t, y) in axes.items():
            fit = best_lag_fit(
                cmd_t,
                cmd,
                y_t,
                y,
                threshold=threshold,
                lag_min_s=lag_min_s,
                lag_max_s=lag_max_s,
                lag_step_s=lag_step_s,
            )
            fit["signed"] = signed_fit(cmd_t, cmd, y_t, y, lag_s=float(fit.get("lag_s", 0.0)), threshold=threshold)
            fits[name] = fit
        out["rc_override_axis_fits"] = fits
        out["rc_override_channel_stats"] = {
            f"ch{i + 1}": scalar_stats(rc_override_t, rc_norm[:, i]) for i in range(rc_norm.shape[1])
        }

    if rc_out.size:
        rc_out_norm = normalize_rc_out(rc_out)
        thr = rc_out_thruster_commands(rc_out_norm)
        out["rc_out_thruster_command_stats"] = {
            name: scalar_stats(rc_out_t, values) for name, values in thr.items()
        }
        if dvl_vel.size:
            resampled = resample_matrix(rc_out_t, rc_out_norm, dvl_t)
            out["rc_out_vs_dvl_linear_fit"] = {
                "surge_ch_mix_note": "rc_out channels are motor outputs; per-thruster force is underdetermined without the simulator allocation model.",
                "dvl_x_from_each_servo": {
                    f"servo_{idx + 1}": best_lag_fit(
                        dvl_t,
                        resampled[:, idx],
                        dvl_t,
                        dvl_vel[:, 0],
                        threshold=threshold,
                        lag_min_s=0.0,
                        lag_max_s=0.0,
                        lag_step_s=1.0,
                    )
                    for idx in range(min(8, resampled.shape[1]))
                },
            }

    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--threshold", type=float, default=0.08)
    parser.add_argument("--lag-min-s", type=float, default=-1.0)
    parser.add_argument("--lag-max-s", type=float, default=1.0)
    parser.add_argument("--lag-step-s", type=float, default=0.05)
    args = parser.parse_args()

    bags = discover_bags(args.root)
    if not bags:
        raise SystemExit(f"No usable bags found under {args.root}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "script": Path(__file__).name,
        "root": str(args.root),
        "threshold": float(args.threshold),
        "lag_search_s": [float(args.lag_min_s), float(args.lag_max_s), float(args.lag_step_s)],
        "bags": {
            db_path.parent.name: analyze_bag(
                db_path,
                threshold=args.threshold,
                lag_min_s=args.lag_min_s,
                lag_max_s=args.lag_max_s,
                lag_step_s=args.lag_step_s,
            )
            for db_path in bags
        },
    }
    out_path = args.output_dir / "thruster_fit_summary.json"
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False))
    print(f"[thruster-fit] wrote {out_path}")


if __name__ == "__main__":
    main()

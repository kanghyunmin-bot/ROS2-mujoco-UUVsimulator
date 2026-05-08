from __future__ import annotations

import argparse
import json
import sqlite3
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
ROOT = Path(__file__).resolve().parents[2]

from analyze_april1_real_bags import read_bag, scalar_stats, vector_stats  # noqa: E402
from compare_closed_loop_april1_replay import (  # noqa: E402
    fit_gain_offset,
    operational_confidence_no_depth,
    pressure_depth_from_static,
    velocity_gyro_score_no_depth,
)
from replay_april1_rc_override_closed_loop import (  # noqa: E402
    RC_OVERRIDE_SPAN,
    load_joy_node_sequence,
    load_rc_sequence,
)
from fit_thruster_params_from_april1 import normalize_rc_out, rc_out_thruster_commands  # noqa: E402


DEFAULT_REAL_BAG = Path(
    ROOT / "real_robot_ros_bag/extracted_2026_04_01/"
    "bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_0.db3"
)
DEFAULT_SIM_BAG = Path(
    ROOT / "document/docsource/"
    "closed_loop_bar30_external_from_start_20260504/sim_bag/sim_bag_0.db3"
)
DEFAULT_OUT = Path(
    ROOT / "document/docsource/"
    "closed_loop_bar30_external_from_start_20260504/command_mapped_diagnostics"
)


def parse_windows(raw: str) -> list[tuple[float, float]]:
    windows: list[tuple[float, float]] = []
    for item in raw.split(","):
        if not item.strip():
            continue
        a, b = item.split(":", 1)
        start = float(a)
        end = float(b)
        if end <= start:
            raise ValueError(f"bad window {item!r}")
        windows.append((start, end))
    return windows


def parse_signs(raw: str) -> np.ndarray:
    signs = np.asarray([float(x.strip()) for x in raw.split(",") if x.strip()], dtype=float)
    if signs.size != 3:
        raise ValueError("--sim-dvl-signs must contain three comma-separated values")
    return signs


def phase_changes(db_path: Path, topic: str = "/measurement/phase") -> list[tuple[float, str]]:
    conn = sqlite3.connect(str(db_path))
    try:
        row = conn.execute("select id, type from topics where name = ?", (topic,)).fetchone()
        if row is None:
            return []
        topic_id, type_name = row
        msg_cls = get_message(str(type_name))
        t0_ns = conn.execute("select min(timestamp) from messages").fetchone()[0]
        last_value: str | None = None
        changes: list[tuple[float, str]] = []
        for timestamp_ns, blob in conn.execute(
            "select timestamp, data from messages where topic_id = ? order by timestamp",
            (int(topic_id),),
        ):
            msg = deserialize_message(bytes(blob), msg_cls)
            value = str(getattr(msg, "data", ""))
            if value != last_value:
                changes.append(((int(timestamp_ns) - int(t0_ns or 0)) * 1.0e-9, value))
                last_value = value
        return changes
    finally:
        conn.close()


def closed_loop_phase_span(changes: list[tuple[float, str]]) -> tuple[float, float]:
    for idx, (t, value) in enumerate(changes):
        if value == "closed_loop_replay":
            end = changes[idx + 1][0] if idx + 1 < len(changes) else float("inf")
            return float(t), float(end)
    raise RuntimeError("sim bag has no /measurement/phase='closed_loop_replay' span")


def first_array(data, keys: list[str], dims: int | None = None) -> tuple[np.ndarray, np.ndarray, str]:
    for key in keys:
        t, values = data.array(key, dims)
        if values.size:
            return t, values, key
    empty = np.empty((0, dims), dtype=float) if dims is not None else np.empty(0, dtype=float)
    return np.empty(0, dtype=float), empty, ""


def unique_time_map(sim_t: np.ndarray, real_t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    sim_t = np.asarray(sim_t, dtype=float).reshape(-1)
    real_t = np.asarray(real_t, dtype=float).reshape(-1)
    n = min(sim_t.size, real_t.size)
    sim_t = sim_t[:n]
    real_t = real_t[:n]
    keep = np.ones(n, dtype=bool)
    keep[1:] = np.diff(sim_t) > 1.0e-9
    return sim_t[keep], real_t[keep]


def map_sim_time_to_real(
    sim_signal_t: np.ndarray,
    sim_rc_t: np.ndarray,
    real_cmd_t: np.ndarray,
) -> np.ndarray:
    sim_map_t, real_map_t = unique_time_map(sim_rc_t, real_cmd_t)
    mapped = np.interp(sim_signal_t, sim_map_t, real_map_t, left=np.nan, right=np.nan)
    outside = (sim_signal_t < sim_map_t[0]) | (sim_signal_t > sim_map_t[-1])
    mapped[outside] = np.nan
    return mapped


def normalized_channel(channels: np.ndarray, idx: int) -> np.ndarray:
    channels = np.asarray(channels, dtype=float)
    if channels.ndim == 1:
        channels = channels.reshape((-1, 1))
    if idx >= channels.shape[1]:
        return np.zeros(channels.shape[0], dtype=float)
    return (channels[:, idx] - 1500.0) / RC_OVERRIDE_SPAN


def rc_out_array(data) -> tuple[np.ndarray, np.ndarray]:
    key = "/mavros/rc/out:channels"
    t = np.asarray(data.series_t.get(key, []), dtype=float)
    raw = list(data.series_v.get(key, []))
    channels = np.zeros((len(raw), 8), dtype=float)
    for idx, row in enumerate(raw):
        values = np.asarray(row, dtype=float).reshape(-1)
        n = min(8, values.size)
        if n:
            channels[idx, :n] = values[:n]
    return t, channels


def rc_out_axis_series(channels: np.ndarray) -> dict[str, np.ndarray]:
    if channels.size == 0:
        return {
            "vertical_mean": np.empty(0, dtype=float),
            "vertical_sum": np.empty(0, dtype=float),
            "vertical_abs": np.empty(0, dtype=float),
            "yaw_abs": np.empty(0, dtype=float),
            "yaw_diff_proxy": np.empty(0, dtype=float),
        }
    norm = normalize_rc_out(channels)
    thr = rc_out_thruster_commands(norm)
    vertical = np.column_stack([thr[name] for name in ("ver_lf", "ver_lr", "ver_rf", "ver_rr")])
    yaw = np.column_stack([thr[name] for name in ("yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr")])
    return {
        "vertical_mean": np.mean(vertical, axis=1),
        "vertical_sum": np.sum(vertical, axis=1),
        "vertical_abs": np.mean(np.abs(vertical), axis=1),
        "yaw_abs": np.mean(np.abs(yaw), axis=1),
        "yaw_diff_proxy": yaw[:, 0] - yaw[:, 1] + yaw[:, 2] - yaw[:, 3],
    }


def derivative(t: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    t = np.asarray(t, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    n = min(t.size, y.size)
    t = t[:n]
    y = y[:n]
    finite = np.isfinite(t) & np.isfinite(y)
    t = t[finite]
    y = y[finite]
    if t.size < 3:
        return t[:0], np.empty(0, dtype=float)
    dy = np.gradient(y) / np.maximum(np.gradient(t), 1.0e-6)
    return t, dy


def compare_scalar_window(
    real_t: np.ndarray,
    real_y: np.ndarray,
    sim_t: np.ndarray,
    sim_y: np.ndarray,
    *,
    start: float,
    end: float,
    lag_min_s: float = -1.0,
    lag_max_s: float = 1.0,
    lag_step_s: float = 0.02,
    prefer: str = "rmse",
    remove_median_offset: bool = False,
) -> dict[str, Any]:
    real_t = np.asarray(real_t, dtype=float).reshape(-1)
    real_y = np.asarray(real_y, dtype=float).reshape(-1)
    sim_t = np.asarray(sim_t, dtype=float).reshape(-1)
    sim_y = np.asarray(sim_y, dtype=float).reshape(-1)
    n = min(real_t.size, real_y.size)
    real_t = real_t[:n]
    real_y = real_y[:n]
    n = min(sim_t.size, sim_y.size)
    sim_t = sim_t[:n]
    sim_y = sim_y[:n]
    real_ok = np.isfinite(real_t) & np.isfinite(real_y)
    sim_ok = np.isfinite(sim_t) & np.isfinite(sim_y)
    real_t = real_t[real_ok]
    real_y = real_y[real_ok]
    sim_t = sim_t[sim_ok]
    sim_y = sim_y[sim_ok]
    if real_t.size < 12 or sim_t.size < 12:
        return {"count": 0}
    order = np.argsort(sim_t)
    sim_t = sim_t[order]
    sim_y = sim_y[order]
    best: dict[str, Any] | None = None
    for lag in np.arange(lag_min_s, lag_max_s + 0.5 * lag_step_s, lag_step_s):
        query_t = real_t - float(lag)
        mask = (real_t >= start) & (real_t <= end) & (query_t >= sim_t[0]) & (query_t <= sim_t[-1])
        if np.sum(mask) < 12:
            continue
        rt = real_t[mask]
        r = real_y[mask]
        s = np.interp(query_t[mask], sim_t, sim_y)
        finite = np.isfinite(r) & np.isfinite(s)
        rt = rt[finite]
        r = r[finite]
        s = s[finite]
        if r.size < 12:
            continue
        residual = r - s
        removed = 0.0
        if remove_median_offset:
            removed = float(np.median(residual))
            residual = residual - removed
        corr = None
        if np.std(r) > 1.0e-12 and np.std(s) > 1.0e-12:
            corr = float(np.corrcoef(r, s)[0, 1])
        candidate = {
            "count": int(r.size),
            "window_s": [float(start), float(end)],
            "lag_s": float(lag),
            "rmse": float(np.sqrt(np.mean(residual * residual))),
            "mae": float(np.mean(np.abs(residual))),
            "bias": float(np.mean(residual)),
            "residual_std": float(np.std(residual)),
            "correlation": corr,
            "removed_median_offset": removed,
            "real": scalar_stats(rt, r),
            "sim": scalar_stats(rt, s),
            "gain_fit_real_from_sim": fit_gain_offset(s, r),
        }
        if best is None:
            best = candidate
            continue
        if prefer == "corr":
            best_corr = -1.0 if best["correlation"] is None else float(best["correlation"])
            cand_corr = -1.0 if candidate["correlation"] is None else float(candidate["correlation"])
            if cand_corr > best_corr + 1.0e-9 or (
                abs(cand_corr - best_corr) <= 1.0e-9 and candidate["rmse"] < best["rmse"]
            ):
                best = candidate
        elif candidate["rmse"] < best["rmse"]:
            best = candidate
    return best or {"count": 0}


def compare_vector_window(
    real_t: np.ndarray,
    real_v: np.ndarray,
    sim_t: np.ndarray,
    sim_v: np.ndarray,
    *,
    names: tuple[str, str, str],
    start: float,
    end: float,
) -> dict[str, Any]:
    if real_v.size == 0 or sim_v.size == 0:
        return {name: {"count": 0} for name in names}
    return {
        name: compare_scalar_window(
            real_t,
            real_v[:, idx],
            sim_t,
            sim_v[:, idx],
            start=start,
            end=end,
            prefer="corr" if name in {"x", "y", "z"} else "rmse",
        )
        for idx, name in enumerate(names)
    }


def command_metrics(
    real_cmd_t: np.ndarray,
    real_channels: np.ndarray,
    sim_cmd_t_mapped: np.ndarray,
    sim_channels: np.ndarray,
    *,
    start: float,
    end: float,
) -> dict[str, Any]:
    out: dict[str, Any] = {}
    axes = {
        "heave_ch3": 2,
        "yaw_ch4": 3,
        "forward_ch5": 4,
        "lateral_ch6": 5,
    }
    for name, idx in axes.items():
        out[name] = compare_scalar_window(
            real_cmd_t,
            normalized_channel(real_channels, idx),
            sim_cmd_t_mapped,
            normalized_channel(sim_channels, idx),
            start=start,
            end=end,
            lag_min_s=-0.1,
            lag_max_s=0.1,
            lag_step_s=0.01,
            prefer="rmse",
        )
    return out


def rc_out_metrics(
    real_t: np.ndarray,
    real_axes: dict[str, np.ndarray],
    sim_t: np.ndarray,
    sim_axes: dict[str, np.ndarray],
    *,
    start: float,
    end: float,
) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for name in ("vertical_mean", "vertical_sum", "vertical_abs", "yaw_abs", "yaw_diff_proxy"):
        out[name] = compare_scalar_window(
            real_t,
            real_axes.get(name, np.empty(0, dtype=float)),
            sim_t,
            sim_axes.get(name, np.empty(0, dtype=float)),
            start=start,
            end=end,
            lag_min_s=-1.0,
            lag_max_s=1.0,
            lag_step_s=0.02,
            prefer="corr",
        )
    return out


def depth_from_pressure_or_pose(data, *, prefer_static_pressure: bool) -> tuple[np.ndarray, np.ndarray, str, dict[str, Any]]:
    depth_t, depth, depth_key = first_array(data, ["/depth/pose:depth_positive_m", "/depth:depth_positive_m"])
    depth = depth.reshape(-1) if depth.size else depth
    pressure_t, pressure_pa, pressure_key = first_array(data, ["/mavros/imu/static_pressure:pressure_pa"])
    calibration: dict[str, Any] = {"status": "unavailable"}
    if prefer_static_pressure and pressure_pa.size:
        p_t, p_depth, calibration = pressure_depth_from_static(
            pressure_t,
            pressure_pa.reshape(-1),
            depth_t,
            depth,
        )
        if p_depth.size:
            return p_t, p_depth.reshape(-1), f"{pressure_key}:pressure_derived_depth_m", calibration
    return depth_t, depth, depth_key, calibration


def plot_command_alignment(
    out_path: Path,
    real_cmd_t: np.ndarray,
    real_channels: np.ndarray,
    sim_cmd_t_raw: np.ndarray,
    sim_cmd_t_mapped: np.ndarray,
    sim_channels: np.ndarray,
) -> None:
    fig, axes = plt.subplots(5, 1, figsize=(13, 9), sharex=False)
    axes_map = [
        ("heave ch3", 2),
        ("yaw ch4", 3),
        ("forward ch5", 4),
        ("lateral ch6", 5),
    ]
    for ax, (name, idx) in zip(axes[:4], axes_map):
        ax.plot(real_cmd_t, normalized_channel(real_channels, idx), "k", lw=0.8, label="real joy-node command")
        ax.plot(sim_cmd_t_mapped, normalized_channel(sim_channels, idx), "r", lw=0.65, alpha=0.8, label="sim /rc/override mapped")
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", fontsize=8)
    n = min(real_cmd_t.size, sim_cmd_t_raw.size)
    drift = sim_cmd_t_raw[:n] - sim_cmd_t_raw[0] - (real_cmd_t[:n] - real_cmd_t[0])
    axes[4].plot(real_cmd_t[:n], drift, color="tab:blue", lw=0.9)
    axes[4].set_ylabel("sim-real drift s")
    axes[4].set_xlabel("real bag time s")
    axes[4].grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_window_overlay(
    out_path: Path,
    *,
    start: float,
    end: float,
    real_cmd_t: np.ndarray,
    real_channels: np.ndarray,
    sim_cmd_t: np.ndarray,
    sim_channels: np.ndarray,
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
    fig, axes = plt.subplots(6, 1, figsize=(14, 12), sharex=True)
    cmd_defs = [("heave", 2), ("forward", 4), ("yaw", 3)]
    for name, idx in cmd_defs:
        axes[0].plot(real_cmd_t, normalized_channel(real_channels, idx), lw=0.7, label=f"real {name}")
    axes[0].set_ylabel("cmd")
    axes[0].legend(loc="upper right", ncol=3, fontsize=8)
    axes[0].grid(True, alpha=0.25)

    if real_dvl.size:
        axes[1].plot(real_dvl_t, real_dvl[:, 0], "k", lw=0.85, label="real vx")
        axes[2].plot(real_dvl_t, real_dvl[:, 1], "k", lw=0.85, label="real vy")
        axes[3].plot(real_dvl_t, real_dvl[:, 2], "k", lw=0.85, label="real vz")
    if sim_dvl.size:
        axes[1].plot(sim_dvl_t, sim_dvl[:, 0], "r", lw=0.75, label="sim vx")
        axes[2].plot(sim_dvl_t, sim_dvl[:, 1], "r", lw=0.75, label="sim vy")
        axes[3].plot(sim_dvl_t, sim_dvl[:, 2], "r", lw=0.75, label="sim vz")
    for idx, ylabel in zip((1, 2, 3), ("vx m/s", "vy m/s", "vz m/s")):
        axes[idx].set_ylabel(ylabel)
        axes[idx].legend(loc="upper right", fontsize=8)
        axes[idx].grid(True, alpha=0.25)

    if real_gyro.size:
        axes[4].plot(real_imu_t, real_gyro[:, 0], color="0.3", lw=0.65, label="real p")
        axes[4].plot(real_imu_t, real_gyro[:, 1], color="0.55", lw=0.65, label="real q")
        axes[4].plot(real_imu_t, real_gyro[:, 2], "k", lw=0.85, label="real r")
    if sim_gyro.size:
        axes[4].plot(sim_imu_t, sim_gyro[:, 0], color="#ff8a8a", lw=0.65, label="sim p")
        axes[4].plot(sim_imu_t, sim_gyro[:, 1], color="#ffb36b", lw=0.65, label="sim q")
        axes[4].plot(sim_imu_t, sim_gyro[:, 2], "r", lw=0.85, label="sim r")
    axes[4].set_ylabel("gyro rad/s")
    axes[4].legend(loc="upper right", ncol=3, fontsize=8)
    axes[4].grid(True, alpha=0.25)

    if real_depth.size:
        axes[5].plot(real_depth_t, real_depth, "k", lw=0.85, label="real Bar30 depth")
    if sim_depth.size:
        axes[5].plot(sim_depth_t, sim_depth, "r", lw=0.85, label="sim Bar30 depth")
    axes[5].set_ylabel("depth m")
    axes[5].set_xlabel("real command time s")
    axes[5].legend(loc="upper right", fontsize=8)
    axes[5].grid(True, alpha=0.25)
    for ax in axes:
        ax.set_xlim(start, end)
    fig.suptitle(f"Command-index mapped real vs sim, {start:.0f}-{end:.0f}s")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)


def plot_confidence_summary(out_path: Path, summary: dict[str, Any]) -> None:
    windows = list(summary["windows"].keys())
    axes = ["dvl_x", "dvl_y", "dvl_z", "gyro_x", "gyro_y", "gyro_z"]
    values = []
    for window in windows:
        conf = summary["windows"][window]["operational_confidence_no_depth"]["axis_metrics"]
        values.append([float(conf.get(axis, {}).get("operational_confidence_percent", 0.0) or 0.0) for axis in axes])
    arr = np.asarray(values, dtype=float)
    fig, ax = plt.subplots(figsize=(12, 4.2))
    im = ax.imshow(arr, vmin=0.0, vmax=100.0, cmap="RdYlGn")
    ax.set_xticks(np.arange(len(axes)))
    ax.set_xticklabels(axes)
    ax.set_yticks(np.arange(len(windows)))
    ax.set_yticklabels(windows)
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            ax.text(j, i, f"{arr[i, j]:.0f}%", ha="center", va="center", fontsize=9, color="black")
    fig.colorbar(im, ax=ax, label="operational confidence %")
    ax.set_title("Depth-excluded confidence by window")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--real-bag", type=Path, default=DEFAULT_REAL_BAG)
    parser.add_argument("--sim-bag", type=Path, default=DEFAULT_SIM_BAG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--command-source", choices=("joy-node", "rc-override"), default="joy-node")
    parser.add_argument("--start-offset-s", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float, default=310.0)
    parser.add_argument("--max-publish-hz", type=float, default=50.0)
    parser.add_argument("--windows", default="60:120,120:180,240:300")
    parser.add_argument(
        "--sim-dvl-signs",
        default="1,1,1",
        help=(
            "Per-axis sign applied to the sim DVL topic before real-vs-sim comparison. "
            "The current real bag and sim /dvl/twist topics match as-published, so the default is raw 1,1,1."
        ),
    )
    parser.add_argument("--depth-source", choices=("pressure", "pose"), default="pressure")
    args = parser.parse_args()

    real_db = args.real_bag
    sim_db = args.sim_bag
    out_dir = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.command_source == "joy-node":
        seq = load_joy_node_sequence(
            real_db,
            start_offset_s=args.start_offset_s,
            duration_s=args.duration_s,
            max_publish_hz=args.max_publish_hz,
        )
    else:
        seq = load_rc_sequence(
            real_db,
            topic="/mavros/rc/override",
            start_offset_s=args.start_offset_s,
            duration_s=args.duration_s,
            max_publish_hz=args.max_publish_hz,
            fill_neutral_first8=True,
        )
    real_cmd_t = seq.origin_s + seq.t
    real_channels = np.asarray(seq.channels, dtype=float)

    real = read_bag(real_db)
    sim = read_bag(sim_db)

    changes = phase_changes(sim_db)
    phase_start, phase_end = closed_loop_phase_span(changes)
    sim_rc_t, sim_channels = sim.array("/mavros/rc/override:channels")
    sim_channels = np.asarray(sim_channels, dtype=float)
    mask = (sim_rc_t >= phase_start - 1.0e-6) & (sim_rc_t <= phase_end + 1.0e-6)
    sim_rc_t = sim_rc_t[mask]
    sim_channels = sim_channels[mask]
    n = min(real_cmd_t.size, sim_rc_t.size)
    if n < 100:
        raise RuntimeError("not enough replay RC samples to build command-index time map")
    real_cmd_t = real_cmd_t[:n]
    real_channels = real_channels[:n]
    sim_rc_t = sim_rc_t[:n]
    sim_channels = sim_channels[:n]
    sim_cmd_t_mapped = map_sim_time_to_real(sim_rc_t, sim_rc_t, real_cmd_t)

    dvl_signs = parse_signs(args.sim_dvl_signs)
    real_dvl_t, real_dvl, real_dvl_key = first_array(real, ["/dvl/twist:linear_m_s"], 3)
    sim_dvl_t_raw, sim_dvl_raw, sim_dvl_key = first_array(
        sim,
        [
            "/dvl/twist:linear_m_s",
            "/dvl/odometry:linear_m_s",
            "/mavros/local_position/velocity_local:linear_m_s",
            "/mavros/local_position/odom:linear_m_s",
        ],
        3,
    )
    sim_dvl_t = map_sim_time_to_real(sim_dvl_t_raw, sim_rc_t, real_cmd_t)
    sim_dvl = sim_dvl_raw * dvl_signs.reshape((1, 3)) if sim_dvl_raw.size else sim_dvl_raw

    real_imu_t, real_gyro, real_gyro_key = first_array(real, ["/mavros/imu/data:gyro_rad_s"], 3)
    sim_imu_t_raw, sim_gyro, sim_gyro_key = first_array(sim, ["/mavros/imu/data:gyro_rad_s"], 3)
    sim_imu_t = map_sim_time_to_real(sim_imu_t_raw, sim_rc_t, real_cmd_t)

    real_depth_t, real_depth, real_depth_key, real_depth_calib = depth_from_pressure_or_pose(
        real,
        prefer_static_pressure=args.depth_source == "pressure",
    )
    sim_depth_t_raw, sim_depth, sim_depth_key, sim_depth_calib = depth_from_pressure_or_pose(
        sim,
        prefer_static_pressure=args.depth_source == "pressure",
    )
    sim_depth_t = map_sim_time_to_real(sim_depth_t_raw, sim_rc_t, real_cmd_t)

    depth_rate_real_t, depth_rate_real = derivative(real_depth_t, real_depth)
    depth_rate_sim_raw_t, depth_rate_sim = derivative(sim_depth_t_raw, sim_depth)
    depth_rate_sim_t = map_sim_time_to_real(depth_rate_sim_raw_t, sim_rc_t, real_cmd_t)

    real_rcout_t, real_rcout = rc_out_array(real)
    sim_rcout_t_raw, sim_rcout = rc_out_array(sim)
    sim_rcout_t = map_sim_time_to_real(sim_rcout_t_raw, sim_rc_t, real_cmd_t)
    real_rcout_axes = rc_out_axis_series(real_rcout)
    sim_rcout_axes = rc_out_axis_series(sim_rcout)

    windows = parse_windows(args.windows)
    summary: dict[str, Any] = {
        "real_bag": str(real_db),
        "sim_bag": str(sim_db),
        "command_source": args.command_source,
        "rc_sequence": {
            "real_origin_s": float(seq.origin_s),
            "sample_count_real_loaded": int(seq.t.size),
            "sample_count_used": int(n),
            "sim_phase_start_s": float(phase_start),
            "sim_phase_end_s": float(phase_end),
            "sim_replay_duration_s": float(sim_rc_t[-1] - sim_rc_t[0]),
            "real_replay_duration_s": float(real_cmd_t[-1] - real_cmd_t[0]),
            "duration_ratio_sim_over_real": float((sim_rc_t[-1] - sim_rc_t[0]) / max(real_cmd_t[-1] - real_cmd_t[0], 1e-9)),
            "sim_minus_real_drift_end_s": float((sim_rc_t[-1] - sim_rc_t[0]) - (real_cmd_t[-1] - real_cmd_t[0])),
        },
        "keys": {
            "real_dvl": real_dvl_key,
            "sim_dvl": sim_dvl_key,
            "sim_dvl_signs_applied": [float(x) for x in dvl_signs],
            "real_gyro": real_gyro_key,
            "sim_gyro": sim_gyro_key,
            "real_depth": real_depth_key,
            "sim_depth": sim_depth_key,
            "rcout_axis_note": (
                "RCOUT axes are derived from the shared ArduSub vectored-6DOF servo map. "
                "vertical_mean/sum compare signed vertical motor output; yaw_diff_proxy compares horizontal differential output."
            ),
        },
        "counts": {
            "real_rcout": int(real_rcout_t.size),
            "sim_rcout": int(sim_rcout_t_raw.size),
        },
        "pressure_depth_calibration": {
            "real": real_depth_calib,
            "sim": sim_depth_calib,
        },
        "distributions": {
            "real_dvl": vector_stats(real_dvl_t, real_dvl, ("x", "y", "z")) if real_dvl.size else {},
            "sim_dvl_corrected": vector_stats(sim_dvl_t, sim_dvl, ("x", "y", "z")) if sim_dvl.size else {},
            "real_gyro": vector_stats(real_imu_t, real_gyro, ("x", "y", "z")) if real_gyro.size else {},
            "sim_gyro": vector_stats(sim_imu_t, sim_gyro, ("x", "y", "z")) if sim_gyro.size else {},
        },
        "windows": {},
    }

    plot_command_alignment(out_dir / "command_alignment_timewarp.png", real_cmd_t, real_channels, sim_rc_t, sim_cmd_t_mapped, sim_channels)

    for start, end in windows:
        window_key = f"{start:.0f}_{end:.0f}"
        window_dir = out_dir / f"window_{window_key}"
        window_dir.mkdir(parents=True, exist_ok=True)
        cmd_metrics = command_metrics(
            real_cmd_t,
            real_channels,
            sim_cmd_t_mapped,
            sim_channels,
            start=start,
            end=end,
        )
        motor_metrics = rc_out_metrics(
            real_rcout_t,
            real_rcout_axes,
            sim_rcout_t,
            sim_rcout_axes,
            start=start,
            end=end,
        )
        dvl_metrics = compare_vector_window(
            real_dvl_t,
            real_dvl,
            sim_dvl_t,
            sim_dvl,
            names=("x", "y", "z"),
            start=start,
            end=end,
        )
        gyro_metrics = compare_vector_window(
            real_imu_t,
            real_gyro,
            sim_imu_t,
            sim_gyro,
            names=("x", "y", "z"),
            start=start,
            end=end,
        )
        normalized_score = velocity_gyro_score_no_depth(dvl_metrics, gyro_metrics)
        confidence = operational_confidence_no_depth(dvl_metrics, gyro_metrics, normalized_score)
        depth_metrics = compare_scalar_window(
            real_depth_t,
            real_depth,
            sim_depth_t,
            sim_depth,
            start=start,
            end=end,
            lag_min_s=-1.0,
            lag_max_s=1.0,
            lag_step_s=0.02,
            prefer="rmse",
        )
        depth_shape_metrics = compare_scalar_window(
            real_depth_t,
            real_depth,
            sim_depth_t,
            sim_depth,
            start=start,
            end=end,
            lag_min_s=-1.0,
            lag_max_s=1.0,
            lag_step_s=0.02,
            prefer="corr",
            remove_median_offset=True,
        )
        depth_rate_metrics = compare_scalar_window(
            depth_rate_real_t,
            depth_rate_real,
            depth_rate_sim_t,
            depth_rate_sim,
            start=start,
            end=end,
            lag_min_s=-1.0,
            lag_max_s=1.0,
            lag_step_s=0.02,
            prefer="corr",
        )
        summary["windows"][window_key] = {
            "command_metrics": cmd_metrics,
            "rcout_metrics": motor_metrics,
            "dvl_velocity_metrics": dvl_metrics,
            "imu_gyro_metrics": gyro_metrics,
            "depth_metrics": depth_metrics,
            "depth_shape_metrics_offset_removed": depth_shape_metrics,
            "depth_rate_metrics": depth_rate_metrics,
            "velocity_gyro_score_no_depth": normalized_score,
            "operational_confidence_no_depth": confidence,
        }
        plot_window_overlay(
            window_dir / f"core_overlay_command_mapped_{window_key}.png",
            start=start,
            end=end,
            real_cmd_t=real_cmd_t,
            real_channels=real_channels,
            sim_cmd_t=sim_cmd_t_mapped,
            sim_channels=sim_channels,
            real_dvl_t=real_dvl_t,
            real_dvl=real_dvl,
            sim_dvl_t=sim_dvl_t,
            sim_dvl=sim_dvl,
            real_imu_t=real_imu_t,
            real_gyro=real_gyro,
            sim_imu_t=sim_imu_t,
            sim_gyro=sim_gyro,
            real_depth_t=real_depth_t,
            real_depth=real_depth,
            sim_depth_t=sim_depth_t,
            sim_depth=sim_depth,
        )

    plot_confidence_summary(out_dir / "confidence_summary.png", summary)
    (out_dir / "command_mapped_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"[command-mapped] wrote {out_dir}")
    print(
        "[command-mapped] replay drift end: "
        f"{summary['rc_sequence']['sim_minus_real_drift_end_s']:+.3f}s "
        f"(ratio={summary['rc_sequence']['duration_ratio_sim_over_real']:.6f})"
    )


if __name__ == "__main__":
    main()

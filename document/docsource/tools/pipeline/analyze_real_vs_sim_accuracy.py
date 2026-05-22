from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


REAL_DEFAULT = Path("real_robot_ros_bag")
SIM_DEFAULT = Path("document/docsource/measurements/mavros_step_test_current_heavefix_20260406_042120/bag")
OUT_DEFAULT = Path("document/docsource/runs/rosbag/real_vs_sim_analysis")


@dataclass
class BagExtract:
    label: str
    uri: Path
    topics: dict[str, str]
    frames: dict[str, dict[str, int]]
    phases: list[tuple[str, float, float]]
    series: dict[str, dict[str, np.ndarray]]


def quat_to_euler_xyzw(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def norm_rows(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return np.array([], dtype=float)
    return np.linalg.norm(values, axis=1)


def moving_average(values: np.ndarray, samples: int) -> np.ndarray:
    if values.size == 0 or samples <= 1:
        return values.copy()
    samples = min(samples, max(1, values.size // 2))
    kernel = np.ones(samples, dtype=float) / samples
    padded = np.pad(values.astype(float), (samples // 2, samples - 1 - samples // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def robust_highpass_std(t: np.ndarray, y: np.ndarray, window_s: float = 2.0) -> float | None:
    if t.size < 8 or y.size < 8:
        return None
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0.0)]
    if dt.size == 0:
        return None
    samples = max(3, int(round(window_s / float(np.median(dt)))))
    trend = moving_average(y, samples)
    residual = y - trend
    q1, q3 = np.percentile(residual, [25, 75])
    iqr = q3 - q1
    if iqr > 1e-12:
        mask = (residual >= q1 - 3.0 * iqr) & (residual <= q3 + 3.0 * iqr)
        residual = residual[mask]
    if residual.size < 3:
        return None
    return float(np.std(residual))


def scalar_stats(t: np.ndarray, y: np.ndarray) -> dict[str, Any]:
    out: dict[str, Any] = {"count": int(y.size)}
    if y.size == 0:
        return out
    finite = np.isfinite(y)
    t = t[finite]
    y = y[finite]
    out["count"] = int(y.size)
    if y.size == 0:
        return out
    out.update(
        {
            "duration_s": float(t[-1] - t[0]) if t.size >= 2 else 0.0,
            "mean": float(np.mean(y)),
            "std": float(np.std(y)),
            "rms": float(np.sqrt(np.mean(y * y))),
            "min": float(np.min(y)),
            "p05": float(np.percentile(y, 5)),
            "p50": float(np.percentile(y, 50)),
            "p95": float(np.percentile(y, 95)),
            "max": float(np.max(y)),
            "range": float(np.max(y) - np.min(y)),
            "highpass_std_2s": robust_highpass_std(t, y, 2.0),
        }
    )
    if t.size >= 3:
        dt = np.diff(t)
        dt = dt[np.isfinite(dt) & (dt > 0.0)]
        if dt.size:
            out.update(
                {
                    "median_rate_hz": float(1.0 / np.median(dt)),
                    "median_dt_s": float(np.median(dt)),
                    "dt_jitter_std_s": float(np.std(dt)),
                    "dt_p95_s": float(np.percentile(dt, 95)),
                }
            )
    return out


def vector_stats(t: np.ndarray, values: np.ndarray, names: tuple[str, ...]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    if values.size == 0:
        for name in names:
            out[name] = {"count": 0}
        out["norm"] = {"count": 0}
        return out
    for idx, name in enumerate(names):
        out[name] = scalar_stats(t, values[:, idx])
    out["norm"] = scalar_stats(t, norm_rows(values))
    return out


def add_point(store: dict[str, list], key: str, t: float, values: list[float]) -> None:
    store[f"{key}_t"].append(t)
    store[key].append(values)


def add_scalar(store: dict[str, list], key: str, t: float, value: float) -> None:
    store[f"{key}_t"].append(t)
    store[key].append(value)


def extract_phase_spans(phase_seq: list[tuple[float, str]]) -> list[tuple[str, float, float]]:
    if not phase_seq:
        return []
    spans: list[tuple[str, float, float]] = []
    cur_name = phase_seq[0][1]
    cur_t = phase_seq[0][0]
    for t, name in phase_seq[1:]:
        if name != cur_name:
            spans.append((cur_name, cur_t, t))
            cur_name = name
            cur_t = t
    spans.append((cur_name, cur_t, phase_seq[-1][0]))
    return spans


def read_bag(uri: Path, label: str) -> BagExtract:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(uri), storage_id="sqlite3"), ConverterOptions("", ""))
    topics = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(type_name) for name, type_name in topics.items()}

    raw: dict[str, list] = defaultdict(list)
    frames: dict[str, defaultdict[str, int]] = defaultdict(lambda: defaultdict(int))
    phase_seq: list[tuple[float, str]] = []
    t0_ns: int | None = None

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if t0_ns is None:
            t0_ns = int(timestamp_ns)
        t = (int(timestamp_ns) - t0_ns) * 1e-9
        msg = deserialize_message(data, msg_types[topic])

        if hasattr(msg, "header"):
            frames[topic][str(msg.header.frame_id)] += 1

        if topic == "/mavros/imu/data":
            q = msg.orientation
            roll, pitch, yaw = quat_to_euler_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
            add_point(raw, "imu_rpy_rad", t, [roll, pitch, yaw])
            add_point(
                raw,
                "imu_gyro_rad_s",
                t,
                [
                    float(msg.angular_velocity.x),
                    float(msg.angular_velocity.y),
                    float(msg.angular_velocity.z),
                ],
            )
            add_point(
                raw,
                "imu_accel_m_s2",
                t,
                [
                    float(msg.linear_acceleration.x),
                    float(msg.linear_acceleration.y),
                    float(msg.linear_acceleration.z),
                ],
            )
            raw["imu_gyro_cov_diag"].append(
                [
                    float(msg.angular_velocity_covariance[0]),
                    float(msg.angular_velocity_covariance[4]),
                    float(msg.angular_velocity_covariance[8]),
                ]
            )
            raw["imu_accel_cov_diag"].append(
                [
                    float(msg.linear_acceleration_covariance[0]),
                    float(msg.linear_acceleration_covariance[4]),
                    float(msg.linear_acceleration_covariance[8]),
                ]
            )

        elif topic == "/dvl/twist":
            add_point(
                raw,
                "dvl_vel_m_s",
                t,
                [
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.linear.y),
                    float(msg.twist.twist.linear.z),
                ],
            )
            raw["dvl_cov_diag"].append(
                [
                    float(msg.twist.covariance[0]),
                    float(msg.twist.covariance[7]),
                    float(msg.twist.covariance[14]),
                ]
            )

        elif topic == "/dvl/odometry":
            add_point(
                raw,
                "dvl_vel_m_s",
                t,
                [
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.linear.y),
                    float(msg.twist.twist.linear.z),
                ],
            )
            raw["dvl_cov_diag"].append(
                [
                    float(msg.twist.covariance[0]),
                    float(msg.twist.covariance[7]),
                    float(msg.twist.covariance[14]),
                ]
            )

        elif topic == "/depth/pose":
            add_scalar(raw, "depth_m", t, float(msg.pose.pose.position.z))

        elif topic == "/depth":
            add_scalar(raw, "depth_m", t, float(msg.data))

        elif topic == "/tf":
            for transform in msg.transforms:
                if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                    p = transform.transform.translation
                    q = transform.transform.rotation
                    roll, pitch, yaw = quat_to_euler_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
                    add_point(raw, "pose_xyz_m", t, [float(p.x), float(p.y), float(p.z)])
                    add_point(raw, "pose_rpy_rad", t, [roll, pitch, yaw])

        elif topic == "/mavros/local_position/odom":
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            roll, pitch, yaw = quat_to_euler_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
            add_point(raw, "pose_xyz_m", t, [float(p.x), float(p.y), float(p.z)])
            add_point(raw, "pose_rpy_rad", t, [roll, pitch, yaw])
            add_point(
                raw,
                "odom_vel_m_s",
                t,
                [
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.linear.y),
                    float(msg.twist.twist.linear.z),
                ],
            )

        elif topic == "/measurement/phase":
            text = str(msg.data)
            phase_seq.append((t, text))

    series: dict[str, dict[str, np.ndarray]] = {}
    vector_keys = {
        "imu_rpy_rad",
        "imu_gyro_rad_s",
        "imu_accel_m_s2",
        "dvl_vel_m_s",
        "pose_xyz_m",
        "pose_rpy_rad",
        "odom_vel_m_s",
    }
    scalar_keys = {"depth_m"}
    for key in vector_keys:
        values = np.asarray(raw.get(key, []), dtype=float)
        times = np.asarray(raw.get(f"{key}_t", []), dtype=float)
        if values.size:
            values = values.reshape((-1, 3))
        else:
            values = np.empty((0, 3), dtype=float)
        series[key] = {"t": times, "v": values}
    for key in scalar_keys:
        values = np.asarray(raw.get(key, []), dtype=float)
        times = np.asarray(raw.get(f"{key}_t", []), dtype=float)
        series[key] = {"t": times, "v": values}
    for key in ("dvl_cov_diag", "imu_gyro_cov_diag", "imu_accel_cov_diag"):
        values = np.asarray(raw.get(key, []), dtype=float)
        if values.size:
            values = values.reshape((-1, 3))
        else:
            values = np.empty((0, 3), dtype=float)
        series[key] = {"t": np.array([], dtype=float), "v": values}

    return BagExtract(
        label=label,
        uri=uri,
        topics=topics,
        frames={k: dict(v) for k, v in frames.items()},
        phases=extract_phase_spans(phase_seq),
        series=series,
    )


def phase_mask(extract: BagExtract, key: str, selector: str) -> np.ndarray:
    t = extract.series[key]["t"]
    if t.size == 0:
        return np.zeros(0, dtype=bool)
    if not extract.phases:
        return np.ones(t.size, dtype=bool)
    if selector == "active":
        terms = ("step", "leg", "turn")
    elif selector == "neutral":
        terms = ("neutral", "settle", "preflight")
    else:
        return np.ones(t.size, dtype=bool)
    mask = np.zeros(t.size, dtype=bool)
    for name, start, end in extract.phases:
        if any(term in name for term in terms):
            mask |= (t >= start) & (t <= end)
    return mask


def summarize_extract(extract: BagExtract, selector: str = "all") -> dict[str, Any]:
    summary: dict[str, Any] = {
        "label": extract.label,
        "uri": str(extract.uri),
        "topics": extract.topics,
        "frames": extract.frames,
        "phase_count": len(extract.phases),
        "phases": [
            {"name": name, "start": start, "end": end, "duration_s": end - start}
            for name, start, end in extract.phases
        ],
    }
    vector_names = {
        "imu_rpy_rad": ("roll", "pitch", "yaw"),
        "imu_gyro_rad_s": ("gx", "gy", "gz"),
        "imu_accel_m_s2": ("ax", "ay", "az"),
        "dvl_vel_m_s": ("vx", "vy", "vz"),
        "pose_xyz_m": ("x", "y", "z"),
        "pose_rpy_rad": ("roll", "pitch", "yaw"),
        "odom_vel_m_s": ("vx", "vy", "vz"),
    }
    for key, names in vector_names.items():
        t = extract.series[key]["t"]
        v = extract.series[key]["v"]
        mask = phase_mask(extract, key, selector)
        summary[key] = vector_stats(t[mask], v[mask], names) if v.size else vector_stats(t, v, names)
        if key.endswith("_rpy_rad") and v.size:
            v_deg = np.rad2deg(v[mask]) if mask.size else np.rad2deg(v)
            summary[f"{key}_deg"] = vector_stats(t[mask], v_deg, names) if mask.size else vector_stats(t, v_deg, names)
    t_depth = extract.series["depth_m"]["t"]
    y_depth = extract.series["depth_m"]["v"]
    mask_depth = phase_mask(extract, "depth_m", selector)
    summary["depth_m"] = scalar_stats(t_depth[mask_depth], y_depth[mask_depth]) if y_depth.size else scalar_stats(t_depth, y_depth)

    for cov_key in ("dvl_cov_diag", "imu_gyro_cov_diag", "imu_accel_cov_diag"):
        cov = extract.series[cov_key]["v"]
        if cov.size:
            summary[cov_key] = {
                "mean": np.mean(cov, axis=0).astype(float).tolist(),
                "sqrt_mean": np.sqrt(np.maximum(np.mean(cov, axis=0), 0.0)).astype(float).tolist(),
            }
        else:
            summary[cov_key] = None
    return summary


def safe_get(summary: dict[str, Any], path: list[str], default: float | None = None) -> float | None:
    cur: Any = summary
    for item in path:
        if not isinstance(cur, dict) or item not in cur:
            return default
        cur = cur[item]
    try:
        return float(cur)
    except (TypeError, ValueError):
        return default


def ratio(real: float | None, sim: float | None) -> float | None:
    if real is None or sim is None or abs(real) < 1e-12:
        return None
    return float(sim / real)


def build_comparison(real_summary: dict[str, Any], sim_summary: dict[str, Any]) -> dict[str, Any]:
    metrics = {
        "dvl_speed_rms_mps": [["dvl_vel_m_s", "norm", "rms"], "Sim/real DVL speed RMS"],
        "dvl_speed_p95_mps": [["dvl_vel_m_s", "norm", "p95"], "Sim/real DVL speed 95th percentile"],
        "depth_range_m": [["depth_m", "range"], "Depth excursion range"],
        "roll_std_deg": [["imu_rpy_rad_deg", "roll", "std"], "IMU roll standard deviation"],
        "pitch_std_deg": [["imu_rpy_rad_deg", "pitch", "std"], "IMU pitch standard deviation"],
        "gyro_norm_rms_rad_s": [["imu_gyro_rad_s", "norm", "rms"], "Gyro norm RMS"],
        "accel_norm_mean_m_s2": [["imu_accel_m_s2", "norm", "mean"], "Acceleration norm mean"],
        "dvl_vx_noise_hp_mps": [["dvl_vel_m_s", "vx", "highpass_std_2s"], "DVL vx high-pass std"],
        "gyro_gx_noise_hp_rad_s": [["imu_gyro_rad_s", "gx", "highpass_std_2s"], "Gyro gx high-pass std"],
        "depth_noise_hp_m": [["depth_m", "highpass_std_2s"], "Depth high-pass std"],
        "imu_rate_hz": [["imu_gyro_rad_s", "gx", "median_rate_hz"], "IMU median rate"],
        "dvl_rate_hz": [["dvl_vel_m_s", "vx", "median_rate_hz"], "DVL median rate"],
        "depth_rate_hz": [["depth_m", "median_rate_hz"], "Depth median rate"],
    }
    out = {}
    for key, (path, note) in metrics.items():
        rv = safe_get(real_summary, path)
        sv = safe_get(sim_summary, path)
        out[key] = {
            "real": rv,
            "sim": sv,
            "sim_over_real": ratio(rv, sv),
            "note": note,
        }
    return out


def plot_overview(real: BagExtract, sim: BagExtract, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(4, 2, figsize=(13, 11), constrained_layout=True)
    pairs = [
        ("dvl_vel_m_s", "DVL velocity norm [m/s]", lambda v: norm_rows(v)),
        ("depth_m", "Depth [m]", lambda v: v),
        ("imu_gyro_rad_s", "Gyro norm [rad/s]", lambda v: norm_rows(v)),
        ("imu_accel_m_s2", "Accel norm [m/s^2]", lambda v: norm_rows(v)),
    ]
    for row, (key, title, fn) in enumerate(pairs):
        for col, extract in enumerate((real, sim)):
            ax = axes[row][col]
            t = extract.series[key]["t"]
            v = extract.series[key]["v"]
            y = fn(v)
            if t.size and y.size:
                ax.plot(t, y, lw=0.8)
            ax.set_title(f"{extract.label}: {title}")
            ax.set_xlabel("t [s]")
            ax.grid(True, alpha=0.25)
    fig.savefig(output_dir / "real_vs_sim_timeseries_overview.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(10, 8), constrained_layout=True)
    hist_items = [
        ("dvl_vel_m_s", "DVL speed [m/s]", lambda v: norm_rows(v)),
        ("imu_gyro_rad_s", "Gyro norm [rad/s]", lambda v: norm_rows(v)),
        ("imu_accel_m_s2", "Accel norm [m/s^2]", lambda v: norm_rows(v)),
        ("depth_m", "Depth [m]", lambda v: v),
    ]
    for ax, (key, title, fn) in zip(axes.ravel(), hist_items):
        for extract, color in ((real, "#2563eb"), (sim, "#dc2626")):
            v = extract.series[key]["v"]
            y = fn(v)
            if y.size:
                ax.hist(y[np.isfinite(y)], bins=60, alpha=0.45, density=True, label=extract.label, color=color)
        ax.set_title(title)
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.savefig(output_dir / "real_vs_sim_distribution.png", dpi=180)
    plt.close(fig)


def write_report(
    output_dir: Path,
    real_all: dict[str, Any],
    sim_all: dict[str, Any],
    sim_active: dict[str, Any],
    comparison_all: dict[str, Any],
    comparison_active: dict[str, Any],
) -> None:
    def metric_line(name: str, comp: dict[str, Any]) -> str:
        item = comp[name]
        rv = item["real"]
        sv = item["sim"]
        rr = item["sim_over_real"]
        rv_s = "n/a" if rv is None else f"{rv:.6g}"
        sv_s = "n/a" if sv is None else f"{sv:.6g}"
        rr_s = "n/a" if rr is None else f"{rr:.3g}x"
        return f"| `{name}` | {rv_s} | {sv_s} | {rr_s} |"

    lines = [
        "# Real vs MuJoCo Sim Bag Analysis",
        "",
        "## Input Bags",
        f"- Real: `{real_all['uri']}`",
        f"- Sim: `{sim_all['uri']}`",
        "",
        "## Important Limitation",
        "The real bag does not contain actuator/RC command topics or `/measurement/phase`, while the sim bag is a scripted SITL step test. "
        "Therefore this report is a sensor/envelope/timing comparison, not a same-input trajectory RMSE validation.",
        "",
        "## Whole-Bag Comparison",
        "| metric | real | sim | sim/real |",
        "|---|---:|---:|---:|",
    ]
    for name in comparison_all:
        lines.append(metric_line(name, comparison_all))
    lines.extend(
        [
            "",
            "## Sim Active-Phase Comparison",
            "The sim side below is restricted to scripted `step`, `leg`, and `turn` phases when available.",
            "| metric | real | sim active | sim/real |",
            "|---|---:|---:|---:|",
        ]
    )
    for name in comparison_active:
        lines.append(metric_line(name, comparison_active))

    real_topics = ", ".join(f"`{name}`" for name in sorted(real_all["topics"]))
    sim_topics = ", ".join(f"`{name}`" for name in sorted(sim_all["topics"]))
    lines.extend(
        [
            "",
            "## Topic Surface",
            f"- Real topics: {real_topics}",
            f"- Sim topics: {sim_topics}",
            "",
            "## Generated Figures",
            "- `real_vs_sim_timeseries_overview.png`",
            "- `real_vs_sim_distribution.png`",
        ]
    )
    (output_dir / "real_vs_sim_report.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--real-bag", type=Path, default=REAL_DEFAULT)
    parser.add_argument("--sim-bag", type=Path, default=SIM_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUT_DEFAULT)
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    real = read_bag(args.real_bag, "real")
    sim = read_bag(args.sim_bag, "sim_current")

    real_all = summarize_extract(real, "all")
    sim_all = summarize_extract(sim, "all")
    sim_active = summarize_extract(sim, "active")
    sim_neutral = summarize_extract(sim, "neutral")
    comparison_all = build_comparison(real_all, sim_all)
    comparison_active = build_comparison(real_all, sim_active)

    payload = {
        "real_all": real_all,
        "sim_all": sim_all,
        "sim_active": sim_active,
        "sim_neutral": sim_neutral,
        "comparison_all": comparison_all,
        "comparison_active": comparison_active,
        "notes": [
            "No actuator/RC command topics are present in the real bag.",
            "No measurement phase markers are present in the real bag.",
            "Direct same-input trajectory RMSE requires a real bag with command topics and synchronized phase markers, or replay of extracted commands into the simulator.",
        ],
    }
    (output_dir / "real_vs_sim_summary.json").write_text(json.dumps(payload, indent=2))
    plot_overview(real, sim, output_dir)
    write_report(output_dir, real_all, sim_all, sim_active, comparison_all, comparison_active)
    print(f"[analysis] wrote {output_dir / 'real_vs_sim_summary.json'}")
    print(f"[analysis] wrote {output_dir / 'real_vs_sim_report.md'}")
    print(f"[analysis] wrote plots in {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

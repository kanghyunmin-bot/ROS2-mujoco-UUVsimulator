from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Apple SD Gothic Neo", "Helvetica", "Arial", "DejaVu Sans"],
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "figure.titlesize": 12,
        "axes.unicode_minus": False,
    }
)


def quat_to_euler_wxyz(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
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


def read_bag(bag_dir: Path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_dir), storage_id="sqlite3"), ConverterOptions("", ""))
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(type_name) for name, type_name in topic_types.items()}

    records: dict[str, list[tuple[float, object]]] = defaultdict(list)
    t0_ns: int | None = None

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if t0_ns is None:
            t0_ns = timestamp_ns
        msg = deserialize_message(data, msg_types[topic])
        records[topic].append(((timestamp_ns - t0_ns) * 1e-9, msg))
    return records


def extract_phase_spans(records) -> list[tuple[str, float, float]]:
    seq = [(t, msg.data) for t, msg in records.get("/measurement/phase", []) if msg.data]
    if not seq:
        return []
    spans: list[tuple[str, float, float]] = []
    cur_name = seq[0][1]
    cur_t = seq[0][0]
    for t, name in seq[1:]:
        if name != cur_name:
            spans.append((cur_name, cur_t, t))
            cur_name = name
            cur_t = t
    spans.append((cur_name, cur_t, seq[-1][0]))
    return spans


def topic_xy(records, topic, fn):
    xs = []
    ys = []
    for t, msg in records.get(topic, []):
        xs.append(t)
        ys.append(fn(msg))
    return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)


def summarize_phase_metric(t, y, start, end):
    mask = (t >= start) & (t <= end)
    if not np.any(mask):
        return None
    ys = y[mask]
    return {
        "mean": float(np.mean(ys)),
        "max": float(np.max(ys)),
        "min": float(np.min(ys)),
        "final": float(ys[-1]),
    }


def build_summary(records, spans):
    odom_t, surge = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.twist.twist.linear.x))
    imu_t, yaw_rate = topic_xy(records, "/mavros/imu/data", lambda m: float(m.angular_velocity.z))
    depth_t, depth = topic_xy(records, "/depth", lambda m: float(m.data))
    odom_roll_t, roll = topic_xy(
        records,
        "/mavros/local_position/odom",
        lambda m: quat_to_euler_wxyz(
            float(m.pose.pose.orientation.w),
            float(m.pose.pose.orientation.x),
            float(m.pose.pose.orientation.y),
            float(m.pose.pose.orientation.z),
        )[0],
    )
    odom_pitch_t, pitch = topic_xy(
        records,
        "/mavros/local_position/odom",
        lambda m: quat_to_euler_wxyz(
            float(m.pose.pose.orientation.w),
            float(m.pose.pose.orientation.x),
            float(m.pose.pose.orientation.y),
            float(m.pose.pose.orientation.z),
        )[1],
    )

    span_map = {name: (start, end) for name, start, end in spans}
    summary = {}

    for phase_name in ("manual_forward_step", "stabilize_forward_step", "alt_hold_forward_step"):
        if phase_name in span_map:
            summary[phase_name] = {
                "surge_mps": summarize_phase_metric(odom_t, surge, *span_map[phase_name]),
            }

    if "manual_yaw_step" in span_map:
        summary["manual_yaw_step"] = {
            "yaw_rate_rad_s": summarize_phase_metric(imu_t, yaw_rate, *span_map["manual_yaw_step"]),
        }

    if "alt_hold_heave_step" in span_map:
        start, end = span_map["alt_hold_heave_step"]
        depth_metric = summarize_phase_metric(depth_t, depth, start, end)
        if depth_metric is not None:
            mask = (depth_t >= start) & (depth_t <= end)
            depth_metric["delta"] = float(depth[mask][-1] - depth[mask][0])
        summary["alt_hold_heave_step"] = {
            "depth_m": depth_metric,
        }

    if "manual_heave_step" in span_map:
        start, end = span_map["manual_heave_step"]
        depth_metric = summarize_phase_metric(depth_t, depth, start, end)
        if depth_metric is not None:
            mask = (depth_t >= start) & (depth_t <= end)
            depth_metric["delta"] = float(depth[mask][-1] - depth[mask][0])
        summary["manual_heave_step"] = {
            "depth_m": depth_metric,
        }

    if "stabilize_forward_step" in span_map:
        start, end = span_map["stabilize_forward_step"]
        summary.setdefault("stabilize_forward_step", {})
        summary["stabilize_forward_step"]["roll_rad"] = summarize_phase_metric(odom_roll_t, roll, start, end)
        summary["stabilize_forward_step"]["pitch_rad"] = summarize_phase_metric(odom_pitch_t, pitch, start, end)

    return summary


def plot_with_phase_shading(ax, spans, interesting=None):
    colors = {
        "manual": "#dbeafe",
        "stabilize": "#dcfce7",
        "alt_hold": "#fef3c7",
    }
    for name, start, end in spans:
        if interesting and not any(key in name for key in interesting):
            continue
        color = None
        for key, c in colors.items():
            if key in name:
                color = c
                break
        if color is None:
            continue
        ax.axvspan(start, end, color=color, alpha=0.35, linewidth=0)


def _series_window(t: np.ndarray, y: np.ndarray, start: float, end: float, pad: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    mask = (t >= start - pad) & (t <= end + pad)
    return t[mask], y[mask]


def _auto_ylim(arrays: list[np.ndarray], floor: float) -> tuple[float, float]:
    merged = np.concatenate([arr for arr in arrays if arr.size > 0], axis=0)
    ymin = float(np.min(merged))
    ymax = float(np.max(merged))
    span = ymax - ymin
    pad = max(span * 0.18, floor)
    if abs(span) < 1e-12:
        center = 0.5 * (ymin + ymax)
        return center - pad, center + pad
    return ymin - pad, ymax + pad


def _value_near(t: np.ndarray, y: np.ndarray, target: float) -> float | None:
    if t.size == 0 or y.size == 0:
        return None
    idx = int(np.argmin(np.abs(t - target)))
    return float(y[idx])


def _phase_note_text(summary: dict, keys: tuple[str, ...]) -> str:
    lines = []
    for key in keys:
        block = summary.get(key)
        if not block:
            continue
        if key == "manual_forward_step":
            peak = max(abs(block["surge_mps"]["max"]), abs(block["surge_mps"]["min"]))
            lines.append(f"peak |u| = {peak:.5f} m/s")
        elif key == "manual_yaw_step":
            peak = max(abs(block["yaw_rate_rad_s"]["max"]), abs(block["yaw_rate_rad_s"]["min"]))
            lines.append(f"peak |r| = {peak:.6f} rad/s")
        elif key == "alt_hold_heave_step":
            delta_mm = 1000.0 * abs(block["depth_m"]["delta"])
            lines.append(f"depth delta = {delta_mm:.2f} mm")
        elif key == "stabilize_forward_step":
            roll_deg = math.degrees(max(abs(block["roll_rad"]["max"]), abs(block["roll_rad"]["min"])))
            pitch_deg = math.degrees(max(abs(block["pitch_rad"]["max"]), abs(block["pitch_rad"]["min"])))
            lines.append(f"peak |roll| = {roll_deg:.3f} deg")
            lines.append(f"peak |pitch| = {pitch_deg:.3f} deg")
    return "\n".join(lines)


def plot_results(records, spans, output_dir: Path, summary: dict) -> None:
    plots_dir = output_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    span_map = {name: (start, end) for name, start, end in spans}
    manual_peak_u = max(
        abs(float(summary.get("manual_forward_step", {}).get("surge_mps", {}).get("max", 0.0))),
        abs(float(summary.get("manual_forward_step", {}).get("surge_mps", {}).get("min", 0.0))),
    )
    manual_peak_r = max(
        abs(float(summary.get("manual_yaw_step", {}).get("yaw_rate_rad_s", {}).get("max", 0.0))),
        abs(float(summary.get("manual_yaw_step", {}).get("yaw_rate_rad_s", {}).get("min", 0.0))),
    )
    depth_delta_mm = 1000.0 * abs(float(summary.get("alt_hold_heave_step", {}).get("depth_m", {}).get("delta", 0.0)))
    alt_hold_peak_u = max(
        abs(float(summary.get("alt_hold_forward_step", {}).get("surge_mps", {}).get("max", 0.0))),
        abs(float(summary.get("alt_hold_forward_step", {}).get("surge_mps", {}).get("min", 0.0))),
    )
    alt_hold_mean_u = abs(float(summary.get("alt_hold_forward_step", {}).get("surge_mps", {}).get("mean", 0.0)))
    has_material_response = manual_peak_u > 0.05 or manual_peak_r > 0.05 or depth_delta_mm > 5.0
    alt_hold_forward_suppressed = (
        manual_peak_u > 0.25
        and alt_hold_peak_u < (0.4 * manual_peak_u)
        and alt_hold_mean_u < 0.10
    )

    rc_t, rc_forward = topic_xy(records, "/mavros/rc/override", lambda m: (float(m.channels[4]) - 1500.0) / 300.0)
    _, rc_yaw = topic_xy(records, "/mavros/rc/override", lambda m: (float(m.channels[3]) - 1500.0) / 300.0)
    _, rc_heave = topic_xy(records, "/mavros/rc/override", lambda m: (float(m.channels[2]) - 1500.0) / 300.0)
    odom_t, surge = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.twist.twist.linear.x))
    _, sway = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.twist.twist.linear.y))
    _, pos_x = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.pose.pose.position.x))
    _, pos_y = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.pose.pose.position.y))
    odom_roll_t, roll = topic_xy(
        records,
        "/mavros/local_position/odom",
        lambda m: quat_to_euler_wxyz(
            float(m.pose.pose.orientation.w),
            float(m.pose.pose.orientation.x),
            float(m.pose.pose.orientation.y),
            float(m.pose.pose.orientation.z),
        )[0],
    )
    _, pitch = topic_xy(
        records,
        "/mavros/local_position/odom",
        lambda m: quat_to_euler_wxyz(
            float(m.pose.pose.orientation.w),
            float(m.pose.pose.orientation.x),
            float(m.pose.pose.orientation.y),
            float(m.pose.pose.orientation.z),
        )[1],
    )
    imu_t, yaw_rate = topic_xy(records, "/mavros/imu/data", lambda m: float(m.angular_velocity.z))
    depth_t, depth = topic_xy(records, "/depth", lambda m: float(m.data))
    _, heave = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.twist.twist.linear.z))
    _, yaw_deg = topic_xy(
        records,
        "/mavros/local_position/odom",
        lambda m: np.rad2deg(
            quat_to_euler_wxyz(
                float(m.pose.pose.orientation.w),
                float(m.pose.pose.orientation.x),
                float(m.pose.pose.orientation.y),
                float(m.pose.pose.orientation.z),
            )[2]
        ),
    )
    speed_mag = np.sqrt(np.square(surge) + np.square(sway) + np.square(heave))
    alt_start, alt_end = span_map["alt_hold_forward_step"]
    alt_start_surge = _value_near(odom_t, surge, alt_start)
    alt_end_surge = _value_near(odom_t, surge, alt_end)
    pitch_deg = np.rad2deg(pitch)
    alt_start_pitch = _value_near(odom_roll_t, pitch_deg, alt_start)
    alt_end_pitch = _value_near(odom_roll_t, pitch_deg, alt_end)
    alt_start_depth = _value_near(depth_t, depth, alt_start)
    alt_end_depth = _value_near(depth_t, depth, alt_end)
    alt_start_yaw = _value_near(odom_t, yaw_deg, alt_start)
    alt_end_yaw = _value_near(odom_t, yaw_deg, alt_end)
    alt_speed_mean = 0.0
    alt_speed_max = 0.0
    alt_mask = (odom_t >= alt_start) & (odom_t <= alt_end)
    if np.any(alt_mask):
        alt_speed_mean = float(np.mean(speed_mag[alt_mask]))
        alt_speed_max = float(np.max(speed_mag[alt_mask]))

    fig, axes = plt.subplots(2, 2, figsize=(13.0, 9.6), dpi=220, constrained_layout=True)

    start, end = span_map["manual_forward_step"]
    t_cmd, y_cmd = _series_window(rc_t, rc_forward, start, end, pad=1.0)
    t_resp, y_resp = _series_window(odom_t, surge, start, end, pad=1.0)
    ax = axes[0, 0]
    ax.axvspan(start, end, color="#dbeafe", alpha=0.32, linewidth=0)
    ax.plot(t_resp, y_resp, color="#f58518", linewidth=1.8, label="surge [m/s]")
    ax.set_xlim(start - 1.0, end + 1.0)
    ax.set_ylim(_auto_ylim([y_resp], 4e-4))
    ax.set_title("Manual forward step")
    ax.set_ylabel("surge [m/s]")
    ax.grid(True, alpha=0.25)
    ax2 = ax.twinx()
    ax2.step(t_cmd, y_cmd, where="post", color="#64748b", linewidth=1.3, label="RC forward")
    ax2.set_ylim(-0.6, 0.6)
    ax2.set_ylabel("RC cmd")
    ax.text(
        0.03,
        0.96,
        _phase_note_text(summary, ("manual_forward_step",)),
        transform=ax.transAxes,
        va="top",
        fontsize=8.6,
        bbox=dict(boxstyle="round,pad=0.22", facecolor="white", edgecolor="#e2e8f0", alpha=0.95),
    )

    start, end = span_map["manual_yaw_step"]
    t_cmd, y_cmd = _series_window(rc_t, rc_yaw, start, end, pad=1.0)
    t_resp, y_resp = _series_window(imu_t, yaw_rate, start, end, pad=1.0)
    ax = axes[0, 1]
    ax.axvspan(start, end, color="#fee2e2", alpha=0.34, linewidth=0)
    ax.plot(t_resp, y_resp, color="#e45756", linewidth=1.8, label="yaw rate [rad/s]")
    ax.set_xlim(start - 1.0, end + 1.0)
    ax.set_ylim(_auto_ylim([y_resp], 2e-4))
    ax.set_title("Manual yaw step")
    ax.set_ylabel("yaw rate [rad/s]")
    ax.grid(True, alpha=0.25)
    ax2 = ax.twinx()
    ax2.step(t_cmd, y_cmd, where="post", color="#64748b", linewidth=1.3, label="RC yaw")
    ax2.set_ylim(-0.6, 0.6)
    ax2.set_ylabel("RC cmd")
    ax.text(
        0.03,
        0.96,
        _phase_note_text(summary, ("manual_yaw_step",)),
        transform=ax.transAxes,
        va="top",
        fontsize=8.6,
        bbox=dict(boxstyle="round,pad=0.22", facecolor="white", edgecolor="#e2e8f0", alpha=0.95),
    )

    start, end = span_map["alt_hold_heave_step"]
    t_cmd, y_cmd = _series_window(rc_t, rc_heave, start, end, pad=1.0)
    t_resp, y_resp = _series_window(depth_t, depth, start, end, pad=1.0)
    baseline_depth = float(y_resp[0]) if y_resp.size else 0.0
    y_depth_mm = 1000.0 * (y_resp - baseline_depth)
    ax = axes[1, 0]
    ax.axvspan(start, end, color="#fef3c7", alpha=0.34, linewidth=0)
    ax.plot(t_resp, y_depth_mm, color="#72b7b2", linewidth=1.8, label="depth delta [mm]")
    ax.set_xlim(start - 1.0, end + 1.0)
    ax.set_ylim(_auto_ylim([y_depth_mm], 0.7))
    ax.set_title("Alt-hold heave step")
    ax.set_ylabel("depth delta [mm]")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.25)
    ax2 = ax.twinx()
    ax2.step(t_cmd, y_cmd, where="post", color="#64748b", linewidth=1.3, label="RC heave")
    ax2.set_ylim(-0.6, 0.6)
    ax2.set_ylabel("RC cmd")
    ax.text(
        0.03,
        0.96,
        _phase_note_text(summary, ("alt_hold_heave_step",)),
        transform=ax.transAxes,
        va="top",
        fontsize=8.6,
        bbox=dict(boxstyle="round,pad=0.22", facecolor="white", edgecolor="#e2e8f0", alpha=0.95),
    )

    start, end = span_map["stabilize_forward_step"]
    t_cmd, y_cmd = _series_window(rc_t, rc_forward, start, end, pad=1.0)
    t_roll, y_roll = _series_window(odom_roll_t, np.rad2deg(roll), start, end, pad=1.0)
    _, y_pitch = _series_window(odom_roll_t, np.rad2deg(pitch), start, end, pad=1.0)
    ax = axes[1, 1]
    ax.axvspan(start, end, color="#dcfce7", alpha=0.34, linewidth=0)
    ax.plot(t_roll, y_roll, color="#f58518", linewidth=1.7, label="roll [deg]")
    ax.plot(t_roll, y_pitch, color="#54a24b", linewidth=1.7, label="pitch [deg]")
    ax.set_xlim(start - 1.0, end + 1.0)
    ax.set_ylim(_auto_ylim([y_roll, y_pitch], 0.05))
    ax.set_title("Stabilize forward step")
    ax.set_ylabel("attitude [deg]")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.25)
    ax2 = ax.twinx()
    ax2.step(t_cmd, y_cmd, where="post", color="#64748b", linewidth=1.3, label="RC forward")
    ax2.set_ylim(-0.6, 0.6)
    ax2.set_ylabel("RC cmd")
    ax.text(
        0.03,
        0.96,
        _phase_note_text(summary, ("stabilize_forward_step",)),
        transform=ax.transAxes,
        va="top",
        fontsize=8.6,
        bbox=dict(boxstyle="round,pad=0.22", facecolor="white", edgecolor="#e2e8f0", alpha=0.95),
    )
    ax.legend(frameon=False, loc="lower right")

    if has_material_response:
        step_title = "RC shadow-path diagnostic responses from the scripted override bag"
    else:
        step_title = "Measured RC step responses: command present, vehicle response is near zero"
    fig.suptitle(step_title, fontsize=12.5, fontweight="bold")
    fig.savefig(plots_dir / "measured_step_responses.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 6.6), dpi=220, constrained_layout=True)
    ax.plot(pos_y, pos_x, color="#4c78a8", linewidth=1.8)
    ax.scatter(pos_y[:1], pos_x[:1], color="#54a24b", s=40, label="start")
    ax.scatter(pos_y[-1:], pos_x[-1:], color="#e45756", s=40, label="end")
    ax.set_xlabel("Y position [m]")
    ax.set_ylabel("X position [m]")
    ax.set_title("Measured XY trajectory\nfrom the RC step sequence")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.legend(frameon=False)
    fig.savefig(plots_dir / "measured_xy_track.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(11.4, 7.8), dpi=220, constrained_layout=True, sharex=False)
    forward_phases = [
        ("manual_forward_step", "MANUAL", "#dbeafe"),
        ("stabilize_forward_step", "STABILIZE", "#dcfce7"),
        ("alt_hold_forward_step", "ALT_HOLD", "#fef3c7"),
    ]
    for ax, (phase_name, label, shade) in zip(axes, forward_phases):
        start, end = span_map[phase_name]
        t_cmd, y_cmd = _series_window(rc_t, rc_forward, start, end, pad=1.0)
        t_resp, y_resp = _series_window(odom_t, surge, start, end, pad=1.0)
        ax.axvspan(start, end, color=shade, alpha=0.34, linewidth=0)
        ax.plot(t_resp, y_resp, color="#f58518", linewidth=1.8)
        ax.set_xlim(start - 1.0, end + 1.0)
        ax.set_ylim(_auto_ylim([y_resp], 4e-4))
        ax.set_ylabel(f"{label}\nsurge [m/s]")
        ax.grid(True, alpha=0.25)
        ax2 = ax.twinx()
        ax2.step(t_cmd, y_cmd, where="post", color="#64748b", linewidth=1.25)
        ax2.set_ylim(-0.6, 0.6)
        ax2.set_ylabel("RC")
    axes[-1].set_xlabel("Time [s]")
    if has_material_response:
        mode_title = "Forward-step diagnosis by mode under the RC shadow path"
    else:
        mode_title = "Forward step comparison by mode: command exists, surge response stays tiny"
    if alt_hold_forward_suppressed:
        note = (
            "same RC forward = 0.50\n"
            f"surge {alt_start_surge:.4f} -> {alt_end_surge:.4f} m/s\n"
            f"|v| mean/max = {alt_speed_mean:.4f}/{alt_speed_max:.4f} m/s\n"
            f"yaw {alt_start_yaw:.1f} -> {alt_end_yaw:.1f} deg\n"
            f"pitch {alt_start_pitch:.3f} -> {alt_end_pitch:.3f} deg\n"
            f"depth {alt_start_depth:.4f} -> {alt_end_depth:.4f} m\n"
            "ALT_HOLD: reduced forward authority + yaw drift"
        )
        axes[-1].text(
            0.03,
            0.95,
            note,
            transform=axes[-1].transAxes,
            va="top",
            fontsize=8.4,
            bbox=dict(boxstyle="round,pad=0.24", facecolor="#fff7ed", edgecolor="#fdba74", alpha=0.97),
        )
    fig.suptitle(mode_title, fontsize=12.2, fontweight="bold")
    fig.savefig(plots_dir / "measured_mode_comparison.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    records = read_bag(Path(args.bag_dir))
    spans = extract_phase_spans(records)
    summary = build_summary(records, spans)

    plot_results(records, spans, output_dir, summary)

    manifest = {
        "bag_dir": str(Path(args.bag_dir)),
        "phase_spans": [{"name": n, "start": s, "end": e} for n, s, e in spans],
        "summary": summary,
    }
    (output_dir / "measurement_summary.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

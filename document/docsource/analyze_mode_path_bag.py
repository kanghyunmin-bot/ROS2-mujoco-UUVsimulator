from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Apple SD Gothic Neo", "Noto Sans CJK KR", "Arial", "DejaVu Sans"],
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "figure.titlesize": 12,
        "axes.unicode_minus": False,
    }
)

MODE_ORDER = ("manual", "stabilize", "alt_hold", "poshold")
MODE_LABELS = {
    "manual": "MANUAL",
    "stabilize": "STABILIZE",
    "alt_hold": "ALT_HOLD",
    "poshold": "POSHOLD",
}
SEGMENT_ORDER = ("forward_leg", "turn_90", "right_leg")
SEGMENT_COLORS = {
    "forward_leg": "#2563eb",
    "turn_90": "#f59e0b",
    "right_leg": "#16a34a",
}
MODE_TRAJ_COLORS = {
    "manual": "#2563eb",
    "stabilize": "#16a34a",
    "alt_hold": "#f59e0b",
    "poshold": "#dc2626",
}


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


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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


def interp_series(t_src: np.ndarray, y_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    if t_src.size == 0 or y_src.size == 0:
        return np.zeros_like(t_dst, dtype=float)
    if t_src.size == 1:
        return np.full_like(t_dst, float(y_src[0]), dtype=float)
    return np.interp(t_dst, t_src, y_src, left=float(y_src[0]), right=float(y_src[-1]))


def downsample_triplets(t: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray, limit: int = 220) -> list[dict[str, float]]:
    if t.size == 0:
        return []
    idx = np.arange(t.size)
    if t.size > limit:
        idx = np.linspace(0, t.size - 1, limit).astype(int)
    return [
        {"t": float(t[i]), "x": float(x[i]), "y": float(y[i]), "depth": float(z[i])}
        for i in idx
    ]


def compute_mode_metrics(records, spans):
    span_map = {name: (start, end) for name, start, end in spans}

    odom_t, pos_x = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.pose.pose.position.x))
    _, pos_y = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.pose.pose.position.y))
    _, pos_z = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.pose.pose.position.z))
    _, surge = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.twist.twist.linear.x))
    _, sway = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.twist.twist.linear.y))
    _, heave = topic_xy(records, "/mavros/local_position/odom", lambda m: float(m.twist.twist.linear.z))
    _, roll = topic_xy(
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
    _, yaw = topic_xy(
        records,
        "/mavros/local_position/odom",
        lambda m: quat_to_euler_wxyz(
            float(m.pose.pose.orientation.w),
            float(m.pose.pose.orientation.x),
            float(m.pose.pose.orientation.y),
            float(m.pose.pose.orientation.z),
        )[2],
    )
    depth_t, depth = topic_xy(records, "/depth", lambda m: float(m.data))

    depth_on_odom = interp_series(depth_t, depth, odom_t) if depth_t.size else -pos_z

    modes: dict[str, dict[str, object]] = {}
    for mode_key in MODE_ORDER:
        mode_segments = {}
        for segment in SEGMENT_ORDER:
            phase_name = f"{mode_key}_{segment}"
            if phase_name not in span_map:
                continue
            start, end = span_map[phase_name]
            mask = (odom_t >= start) & (odom_t <= end)
            if not np.any(mask):
                continue
            t_seg = odom_t[mask]
            x_seg = pos_x[mask]
            y_seg = pos_y[mask]
            depth_seg = depth_on_odom[mask]
            surge_seg = surge[mask]
            sway_seg = sway[mask]
            heave_seg = heave[mask]
            yaw_seg = yaw[mask]

            dx = float(x_seg[-1] - x_seg[0])
            dy = float(y_seg[-1] - y_seg[0])
            dz = float(depth_seg[-1] - depth_seg[0])
            horiz_distance = float(np.hypot(dx, dy))
            path_length = float(np.sum(np.hypot(np.diff(x_seg), np.diff(y_seg))))

            payload: dict[str, object] = {
                "start_t": float(start),
                "end_t": float(end),
                "duration_s": float(end - start),
                "start_xyz_depth": [float(x_seg[0]), float(y_seg[0]), float(depth_seg[0])],
                "end_xyz_depth": [float(x_seg[-1]), float(y_seg[-1]), float(depth_seg[-1])],
                "delta_xyz_depth": [dx, dy, dz],
                "horizontal_distance_m": horiz_distance,
                "path_length_m": path_length,
                "mean_surge_mps": float(np.mean(surge_seg)),
                "mean_sway_mps": float(np.mean(sway_seg)),
                "mean_heave_mps": float(np.mean(heave_seg)),
                "trajectory_xyz_depth": downsample_triplets(t_seg, x_seg, y_seg, depth_seg),
            }
            if segment == "turn_90":
                yaw_delta = float(math.degrees(wrap_to_pi(float(yaw_seg[-1] - yaw_seg[0]))))
                payload["yaw_delta_deg"] = yaw_delta
                payload["mean_yaw_deg"] = float(np.mean(np.rad2deg(yaw_seg)))
            mode_segments[segment] = payload

        mode_start_name = f"{mode_key}_forward_leg"
        mode_end_name = f"{mode_key}_right_leg"
        if mode_start_name not in span_map or mode_end_name not in span_map:
            continue

        start_t = span_map[mode_start_name][0]
        end_t = span_map[mode_end_name][1]
        mask = (odom_t >= start_t) & (odom_t <= end_t)
        if not np.any(mask):
            continue

        x_all = pos_x[mask]
        y_all = pos_y[mask]
        depth_all = depth_on_odom[mask]
        roll_all = np.rad2deg(roll[mask])
        pitch_all = np.rad2deg(pitch[mask])
        yaw_all = np.unwrap(yaw[mask])
        traj_path_length = float(np.sum(np.sqrt(np.diff(x_all) ** 2 + np.diff(y_all) ** 2 + np.diff(depth_all) ** 2)))

        mode_summary = {
            "start_t": float(start_t),
            "end_t": float(end_t),
            "duration_s": float(end_t - start_t),
            "start_xyz_depth": [float(x_all[0]), float(y_all[0]), float(depth_all[0])],
            "end_xyz_depth": [float(x_all[-1]), float(y_all[-1]), float(depth_all[-1])],
            "delta_xyz_depth": [
                float(x_all[-1] - x_all[0]),
                float(y_all[-1] - y_all[0]),
                float(depth_all[-1] - depth_all[0]),
            ],
            "horizontal_distance_m": float(np.hypot(x_all[-1] - x_all[0], y_all[-1] - y_all[0])),
            "trajectory_path_length_m": traj_path_length,
            "max_abs_roll_deg": float(np.max(np.abs(roll_all))),
            "max_abs_pitch_deg": float(np.max(np.abs(pitch_all))),
            "yaw_change_deg": float(np.degrees(yaw_all[-1] - yaw_all[0])),
            "trajectory_xyz_depth": downsample_triplets(odom_t[mask], x_all, y_all, depth_all),
        }
        modes[mode_key] = {
            "label": MODE_LABELS[mode_key],
            "segments": mode_segments,
            "summary": mode_summary,
        }

    return modes


def plot_mode_3d(modes: dict[str, dict[str, object]], output_path: Path, title: str) -> None:
    fig = plt.figure(figsize=(13.0, 9.8), dpi=220)
    for idx, mode_key in enumerate(MODE_ORDER, start=1):
        if mode_key not in modes:
            continue
        ax = fig.add_subplot(2, 2, idx, projection="3d")
        mode_data = modes[mode_key]
        summary = mode_data["summary"]
        pts = summary["trajectory_xyz_depth"]
        x = np.asarray([p["x"] for p in pts], dtype=float)
        y = np.asarray([p["y"] for p in pts], dtype=float)
        depth = np.asarray([p["depth"] for p in pts], dtype=float)
        start = summary["start_xyz_depth"]
        end = summary["end_xyz_depth"]
        ax.plot(
            x,
            y,
            depth,
            color=MODE_TRAJ_COLORS[mode_key],
            linewidth=2.4,
            solid_capstyle="round",
        )
        ax.scatter([start[0]], [start[1]], [start[2]], color="#111827", s=24)
        ax.scatter([end[0]], [end[1]], [end[2]], color="#dc2626", s=24)
        ax.set_title(
            f"{MODE_LABELS[mode_key]}\n"
            f"path {summary['trajectory_path_length_m']:.2f} m | "
            f"max pitch {summary['max_abs_pitch_deg']:.1f} deg",
            fontsize=9.5,
            pad=8,
        )
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Depth [m]")
        ax.tick_params(axis="both", which="major", labelsize=8, pad=1)
        ax.view_init(elev=22, azim=-62)
    fig.subplots_adjust(top=0.88, bottom=0.10, left=0.04, right=0.98, wspace=0.08, hspace=0.16)
    fig.legend(
        [
            Line2D([0], [0], color="#111827", marker="o", linestyle="None", markersize=5),
            Line2D([0], [0], color="#dc2626", marker="o", linestyle="None", markersize=5),
        ],
        ["Start", "End"],
        loc="lower center",
        bbox_to_anchor=(0.5, 0.02),
        ncol=2,
        frameon=False,
    )
    fig.suptitle(title, y=0.96, fontsize=12)
    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_mode_xy(modes: dict[str, dict[str, object]], output_path: Path, title: str) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.4, 9.6), dpi=220)
    for ax, mode_key in zip(axes.ravel(), MODE_ORDER):
        if mode_key not in modes:
            ax.axis("off")
            continue
        mode_data = modes[mode_key]
        for segment in SEGMENT_ORDER:
            seg = mode_data["segments"].get(segment)
            if not seg:
                continue
            pts = seg["trajectory_xyz_depth"]
            x = np.asarray([p["x"] for p in pts], dtype=float)
            y = np.asarray([p["y"] for p in pts], dtype=float)
            label = {"forward_leg": "Forward", "turn_90": "Turn", "right_leg": "Right"}[segment]
            ax.plot(y, x, color=SEGMENT_COLORS[segment], linewidth=2.0, label=label)
        summary = mode_data["summary"]
        start = summary["start_xyz_depth"]
        end = summary["end_xyz_depth"]
        ax.scatter([start[1]], [start[0]], color="#111827", s=30, label="start")
        ax.scatter([end[1]], [end[0]], color="#dc2626", s=30, label="end")
        ax.set_title(
            f"{MODE_LABELS[mode_key]}\n"
            f"turn {mode_data['segments']['turn_90']['yaw_delta_deg']:.1f} deg | "
            f"path {summary['trajectory_path_length_m']:.2f} m",
            fontsize=9.5,
            pad=8,
        )
        ax.set_xlabel("Y [m]")
        ax.set_ylabel("X [m]")
        ax.grid(True, alpha=0.25)
        ax.axis("equal")
    fig.subplots_adjust(top=0.88, bottom=0.12, left=0.07, right=0.98, wspace=0.24, hspace=0.32)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.02), ncol=5, frameon=False)
    fig.suptitle(title, y=0.96, fontsize=12)
    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_mode_metrics(modes: dict[str, dict[str, object]], output_path: Path, title: str) -> None:
    labels = [MODE_LABELS[m] for m in MODE_ORDER if m in modes]
    forward = [modes[m]["segments"]["forward_leg"]["horizontal_distance_m"] for m in MODE_ORDER if m in modes]
    yaw_delta = [abs(modes[m]["segments"]["turn_90"]["yaw_delta_deg"]) for m in MODE_ORDER if m in modes]
    right = [modes[m]["segments"]["right_leg"]["horizontal_distance_m"] for m in MODE_ORDER if m in modes]
    pitch = [modes[m]["summary"]["max_abs_pitch_deg"] for m in MODE_ORDER if m in modes]

    x = np.arange(len(labels))
    width = 0.19

    fig, ax = plt.subplots(figsize=(11.8, 5.8), dpi=220)
    ax.bar(x - 1.5 * width, forward, width=width, label="Forward leg [m]", color="#2563eb")
    ax.bar(x - 0.5 * width, yaw_delta, width=width, label="Turn [deg]", color="#f59e0b")
    ax.bar(x + 0.5 * width, right, width=width, label="Right leg [m]", color="#16a34a")
    ax.bar(x + 1.5 * width, pitch, width=width, label="Max |pitch| [deg]", color="#dc2626")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_title(title, fontsize=11, pad=14)
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(frameon=False, ncol=4, loc="upper center", bbox_to_anchor=(0.5, 1.12))
    fig.subplots_adjust(top=0.78, bottom=0.12, left=0.07, right=0.98)
    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--engine-label", default="current")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    plots_dir = output_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    records = read_bag(Path(args.bag_dir))
    spans = extract_phase_spans(records)
    modes = compute_mode_metrics(records, spans)

    plot_mode_3d(
        modes,
        plots_dir / "mode_path_3d.png",
        f"{args.engine_label} actual 3D path: forward → 90° turn → right",
    )
    plot_mode_xy(
        modes,
        plots_dir / "mode_path_xy.png",
        f"{args.engine_label} actual XY path by mode",
    )
    plot_mode_metrics(
        modes,
        plots_dir / "mode_path_metrics.png",
        f"{args.engine_label} mode-path summary metrics",
    )

    manifest = {
        "bag_dir": str(Path(args.bag_dir)),
        "engine_label": args.engine_label,
        "phase_spans": [{"name": n, "start": s, "end": e} for n, s, e in spans],
        "modes": modes,
    }
    (output_dir / "mode_path_summary.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

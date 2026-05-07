from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


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


def load_summary(path: Path) -> dict:
    return json.loads(path.read_text())


def plot_engine_xy(custom: dict, ellipsoid: dict, output_path: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.2, 9.4), dpi=220)
    for ax, mode_key in zip(axes.ravel(), MODE_ORDER):
        c_mode = custom["modes"].get(mode_key)
        e_mode = ellipsoid["modes"].get(mode_key)
        if not c_mode or not e_mode:
            ax.axis("off")
            continue

        c_pts = c_mode["summary"]["trajectory_xyz_depth"]
        e_pts = e_mode["summary"]["trajectory_xyz_depth"]
        c_x = np.asarray([p["x"] for p in c_pts], dtype=float)
        c_y = np.asarray([p["y"] for p in c_pts], dtype=float)
        e_x = np.asarray([p["x"] for p in e_pts], dtype=float)
        e_y = np.asarray([p["y"] for p in e_pts], dtype=float)

        ax.plot(c_y, c_x, color="#dc2626", linewidth=2.1, label="Custom")
        ax.plot(e_y, e_x, color="#2563eb", linewidth=2.1, label="Ellipsoid")
        ax.scatter([c_y[0]], [c_x[0]], color="#111827", s=20)
        ax.scatter([c_y[-1]], [c_x[-1]], color="#dc2626", s=26)
        ax.scatter([e_y[-1]], [e_x[-1]], color="#2563eb", s=26)
        ax.set_title(MODE_LABELS[mode_key])
        ax.set_xlabel("Y [m]")
        ax.set_ylabel("X [m]")
        ax.axis("equal")
        ax.grid(True, alpha=0.25)

    fig.subplots_adjust(top=0.88, bottom=0.11, left=0.07, right=0.98, wspace=0.22, hspace=0.26)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.02), ncol=2, frameon=False)
    fig.suptitle("Actual engine comparison: XY path by mode", y=0.96, fontsize=12)
    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_engine_3d(custom: dict, ellipsoid: dict, output_path: Path) -> None:
    fig = plt.figure(figsize=(12.4, 9.6), dpi=220)
    for idx, mode_key in enumerate(MODE_ORDER, start=1):
        c_mode = custom["modes"].get(mode_key)
        e_mode = ellipsoid["modes"].get(mode_key)
        ax = fig.add_subplot(2, 2, idx, projection="3d")
        if not c_mode or not e_mode:
            ax.set_axis_off()
            continue

        c_pts = c_mode["summary"]["trajectory_xyz_depth"]
        e_pts = e_mode["summary"]["trajectory_xyz_depth"]
        c_x = np.asarray([p["x"] for p in c_pts], dtype=float)
        c_y = np.asarray([p["y"] for p in c_pts], dtype=float)
        c_d = np.asarray([p["depth"] for p in c_pts], dtype=float)
        e_x = np.asarray([p["x"] for p in e_pts], dtype=float)
        e_y = np.asarray([p["y"] for p in e_pts], dtype=float)
        e_d = np.asarray([p["depth"] for p in e_pts], dtype=float)

        ax.plot(c_x, c_y, c_d, color="#dc2626", linewidth=2.2, label="Custom" if idx == 1 else None)
        ax.plot(e_x, e_y, e_d, color="#2563eb", linewidth=2.2, label="Ellipsoid" if idx == 1 else None)
        ax.scatter([c_x[0]], [c_y[0]], [c_d[0]], color="#111827", s=18)
        ax.scatter([c_x[-1]], [c_y[-1]], [c_d[-1]], color="#dc2626", s=24)
        ax.scatter([e_x[-1]], [e_y[-1]], [e_d[-1]], color="#2563eb", s=24)
        ax.set_title(MODE_LABELS[mode_key], fontsize=10, pad=8)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Depth [m]")
        ax.tick_params(axis="both", which="major", labelsize=8, pad=1)
        ax.view_init(elev=22, azim=-62)

    fig.subplots_adjust(top=0.88, bottom=0.10, left=0.04, right=0.98, wspace=0.08, hspace=0.18)
    handles, labels = fig.axes[0].get_legend_handles_labels() if fig.axes else ([], [])
    if handles:
        fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.02), ncol=2, frameon=False)
    fig.suptitle("Actual engine comparison: 3D trajectory by mode", y=0.96, fontsize=12)
    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_engine_metrics(custom: dict, ellipsoid: dict, output_path: Path) -> None:
    labels = [MODE_LABELS[m] for m in MODE_ORDER if m in custom["modes"] and m in ellipsoid["modes"]]
    x = np.arange(len(labels))
    width = 0.16

    c_forward = [custom["modes"][m]["segments"]["forward_leg"]["horizontal_distance_m"] for m in MODE_ORDER if m in custom["modes"] and m in ellipsoid["modes"]]
    e_forward = [ellipsoid["modes"][m]["segments"]["forward_leg"]["horizontal_distance_m"] for m in MODE_ORDER if m in custom["modes"] and m in ellipsoid["modes"]]
    c_right = [custom["modes"][m]["segments"]["right_leg"]["horizontal_distance_m"] for m in MODE_ORDER if m in custom["modes"] and m in ellipsoid["modes"]]
    e_right = [ellipsoid["modes"][m]["segments"]["right_leg"]["horizontal_distance_m"] for m in MODE_ORDER if m in custom["modes"] and m in ellipsoid["modes"]]
    c_pitch = [custom["modes"][m]["summary"]["max_abs_pitch_deg"] for m in MODE_ORDER if m in custom["modes"] and m in ellipsoid["modes"]]
    e_pitch = [ellipsoid["modes"][m]["summary"]["max_abs_pitch_deg"] for m in MODE_ORDER if m in custom["modes"] and m in ellipsoid["modes"]]

    fig, ax = plt.subplots(figsize=(12.0, 5.8), dpi=220)
    ax.bar(x - 2.5 * width, c_forward, width=width, color="#f87171", label="Custom forward [m]")
    ax.bar(x - 1.5 * width, e_forward, width=width, color="#60a5fa", label="Ellipsoid forward [m]")
    ax.bar(x - 0.5 * width, c_right, width=width, color="#ef4444", label="Custom right [m]")
    ax.bar(x + 0.5 * width, e_right, width=width, color="#2563eb", label="Ellipsoid right [m]")
    ax.bar(x + 1.5 * width, c_pitch, width=width, color="#fb7185", label="Custom max |pitch| [deg]")
    ax.bar(x + 2.5 * width, e_pitch, width=width, color="#0ea5e9", label="Ellipsoid max |pitch| [deg]")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_title("Actual engine comparison: path tracking distance and pitch disturbance", fontsize=11, pad=14)
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(frameon=False, ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.12))
    fig.subplots_adjust(top=0.78, bottom=0.12, left=0.07, right=0.98)
    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def build_comparison_json(custom: dict, ellipsoid: dict, output_path: Path) -> None:
    payload = {"custom_label": custom["engine_label"], "ellipsoid_label": ellipsoid["engine_label"], "modes": {}}
    for mode_key in MODE_ORDER:
        if mode_key not in custom["modes"] or mode_key not in ellipsoid["modes"]:
            continue
        c_mode = custom["modes"][mode_key]
        e_mode = ellipsoid["modes"][mode_key]
        payload["modes"][mode_key] = {
            "forward_distance_delta_m": float(
                e_mode["segments"]["forward_leg"]["horizontal_distance_m"]
                - c_mode["segments"]["forward_leg"]["horizontal_distance_m"]
            ),
            "right_distance_delta_m": float(
                e_mode["segments"]["right_leg"]["horizontal_distance_m"]
                - c_mode["segments"]["right_leg"]["horizontal_distance_m"]
            ),
            "pitch_delta_deg": float(
                e_mode["summary"]["max_abs_pitch_deg"] - c_mode["summary"]["max_abs_pitch_deg"]
            ),
            "yaw_delta_deg_custom": float(c_mode["segments"]["turn_90"]["yaw_delta_deg"]),
            "yaw_delta_deg_ellipsoid": float(e_mode["segments"]["turn_90"]["yaw_delta_deg"]),
        }
    output_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--custom-summary", required=True)
    parser.add_argument("--ellipsoid-summary", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    custom = load_summary(Path(args.custom_summary))
    ellipsoid = load_summary(Path(args.ellipsoid_summary))

    plot_engine_3d(custom, ellipsoid, output_dir / "engine_mode_path_3d_comparison.png")
    plot_engine_xy(custom, ellipsoid, output_dir / "engine_mode_path_xy_comparison.png")
    plot_engine_metrics(custom, ellipsoid, output_dir / "engine_mode_path_metrics.png")
    build_comparison_json(custom, ellipsoid, output_dir / "engine_mode_path_comparison.json")


if __name__ == "__main__":
    main()

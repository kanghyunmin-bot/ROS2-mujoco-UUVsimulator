#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import re
import shutil
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


ROOT = Path(__file__).resolve().parents[2]
DOCSRC_DIR = Path(__file__).resolve().parent
METRICS_DIR = DOCSRC_DIR / "metrics" / "report_inputs"
FIG_DIR = DOCSRC_DIR / "figures_v22_latest"
SIM_DIR = ROOT / "uuv_mujoco" / "v2.2"
CONFIG_PATH = SIM_DIR / "config" / "sim_profiles.json"
CURRENT_SCENE = SIM_DIR / "scenes" / "tank_current_scene.xml"
START_SCRIPT = SIM_DIR / "start_ardusub_sitl_mj311.sh"
LEGACY_MEAS_PATH = METRICS_DIR / "measurement_legacy_mode_path_latest_v30.json"
CURRENT_MEAS_PATH = METRICS_DIR / "measurement_current_mode_path_latest_v30.json"
CURRENT_STEP_SUMMARY_PATH = METRICS_DIR / "measurement_summary_current_heavefix_latest_v30.json"
SUMMARY_JSON_PATH = METRICS_DIR / "uuv_v22_latest_report_metrics.json"
CURRENT_STEP_DIR = DOCSRC_DIR / "measurements" / "mavros_step_test_current_heavefix_20260406_042120"
CURRENT_STEP_PLOT = CURRENT_STEP_DIR / "plots" / "measured_step_responses.png"
CURRENT_MODE_PLOT = CURRENT_STEP_DIR / "plots" / "measured_mode_comparison.png"

MODE_ORDER = ("manual", "stabilize", "alt_hold", "poshold")
MODE_LABELS = {
    "manual": "MANUAL",
    "stabilize": "STABILIZE",
    "alt_hold": "ALT_HOLD",
    "poshold": "POSHOLD",
}
MODE_COLORS = {
    "manual": "#2563eb",
    "stabilize": "#16a34a",
    "alt_hold": "#f59e0b",
    "poshold": "#dc2626",
}
MODEL_COLORS = {
    "legacy": "#7c3aed",
    "current": "#0f766e",
}
LANE_COLORS = {
    "ops": "#eff6ff",
    "autopilot": "#fff1f2",
    "runtime": "#ecfdf5",
    "analysis": "#fffbeb",
}

plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Apple SD Gothic Neo", "Noto Sans CJK KR", "Arial", "DejaVu Sans"],
        "axes.titlesize": 12,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "figure.titlesize": 14,
        "axes.unicode_minus": False,
    }
)


def ensure_dirs() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def load_profiles() -> dict:
    return load_json(CONFIG_PATH)


def load_measurements() -> tuple[dict, dict]:
    return load_json(LEGACY_MEAS_PATH), load_json(CURRENT_MEAS_PATH)


def load_step_summary() -> dict:
    return load_json(CURRENT_STEP_SUMMARY_PATH)


def parse_scene(path: Path) -> dict:
    root = ET.parse(path).getroot()
    base = root.find(".//body[@name='base_link']")
    if base is None:
        raise RuntimeError(f"base_link not found in {path}")

    sites = {}
    for site in base.findall("site"):
        name = site.attrib.get("name", "")
        sites[name] = np.array([float(v) for v in site.attrib.get("pos", "0 0 0").split()], dtype=float)

    actuators = {}
    for motor in root.findall(".//motor"):
        name = motor.attrib.get("name", "")
        actuators[name] = {
            "site": motor.attrib.get("site", ""),
            "gear": np.array([float(v) for v in motor.attrib.get("gear", "0 0 0").split()[:3]], dtype=float),
        }

    fluid_geoms = []
    for geom in base.findall("geom"):
        name = geom.attrib.get("name", "")
        if not name.startswith("fluid_"):
            continue
        size = np.array([float(v) for v in geom.attrib.get("size", "0 0 0").split()], dtype=float)
        volume = float(4.0 * math.pi * size[0] * size[1] * size[2] / 3.0)
        fluid_geoms.append(
            {
                "name": name,
                "pos": np.array([float(v) for v in geom.attrib.get("pos", "0 0 0").split()], dtype=float),
                "size": size,
                "fluidcoef": np.array([float(v) for v in geom.attrib.get("fluidcoef", "0 0 0 0 0").split()], dtype=float),
                "volume_m3": volume,
            }
        )
    return {"sites": sites, "actuators": actuators, "fluid_geoms": fluid_geoms}


def parse_sitl_params(path: Path) -> dict[str, str]:
    text = path.read_text()
    found = re.findall(r'append_param_if_not_overridden "([^"]+)" "([^"]+)"', text)
    return {key: value for key, value in found}


def composite_summary(profile: dict) -> dict:
    components = profile["body_components"]
    total_mass = float(sum(component["mass"] for component in components))
    com = np.zeros(3, dtype=float)
    for comp in components:
        com += float(comp["mass"]) * np.asarray(comp["mass_pos"], dtype=float)
    com /= max(total_mass, 1e-9)

    inertia = np.zeros(3, dtype=float)
    for comp in components:
        a, b, c = [float(v) for v in comp["size"]]
        m = float(comp["mass"])
        self_inertia = np.array(
            [
                m * (b * b + c * c) / 5.0,
                m * (a * a + c * c) / 5.0,
                m * (a * a + b * b) / 5.0,
            ],
            dtype=float,
        )
        offset = np.asarray(comp["mass_pos"], dtype=float) - com
        parallel = m * np.array(
            [
                offset[1] ** 2 + offset[2] ** 2,
                offset[0] ** 2 + offset[2] ** 2,
                offset[0] ** 2 + offset[1] ** 2,
            ],
            dtype=float,
        )
        inertia += self_inertia + parallel

    scale = np.asarray(profile.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]), dtype=float)
    buoyancy_points = profile.get("buoyancy_points", [])
    buoy_center = np.zeros(3, dtype=float)
    if buoyancy_points:
        total_share = sum(float(point["share"]) for point in buoyancy_points)
        for point in buoyancy_points:
            buoy_center += float(point["share"]) * np.asarray(point["pos"], dtype=float)
        buoy_center /= max(total_share, 1e-9)
    buoy_center[0] += float(profile.get("cob_x_offset", 0.0))
    buoy_center[2] += float(profile.get("cob_z_offset", 0.0))
    return {
        "mass_total": total_mass,
        "com": com,
        "inertia_raw": inertia,
        "inertia_scaled": inertia * scale,
        "buoy_center": buoy_center,
    }


def wrapped(text: str, width: int) -> str:
    return textwrap.fill(text, width=width, break_long_words=False)


def percent_reduction(old: float, new: float) -> float:
    if abs(old) <= 1e-12:
        return 0.0
    return (1.0 - (new / old)) * 100.0


def mode_arrays(measurement: dict, mode_key: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pts = measurement["modes"][mode_key]["summary"]["trajectory_xyz_depth"]
    x = np.array([float(p["x"]) for p in pts], dtype=float)
    y = np.array([float(p["y"]) for p in pts], dtype=float)
    depth = np.array([float(p["depth"]) for p in pts], dtype=float)
    return x, y, depth


def add_lane(ax, x: float, y: float, w: float, h: float, label: str, color: str) -> None:
    patch = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.015,rounding_size=0.03",
        linewidth=0.8,
        facecolor=color,
        edgecolor="#cbd5e1",
        zorder=0,
    )
    ax.add_patch(patch)
    ax.text(x + 0.015, y + h - 0.02, label, fontsize=11.2, fontweight="bold", color="#0f172a", va="top")


def add_group_box(
    ax,
    x: float,
    y: float,
    w: float,
    h: float,
    title: str,
    subtitle: str,
    fc: str,
    ec: str = "#bfdbfe",
) -> None:
    patch = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.018,rounding_size=0.04",
        linewidth=1.3,
        facecolor=fc,
        edgecolor=ec,
        zorder=0,
    )
    ax.add_patch(patch)
    cx = x + w * 0.5
    ax.text(cx, y + h - 0.040, title, fontsize=13.6, fontweight="bold", color="#0f172a", va="top", ha="center", zorder=1)
    divider_y = y + h - 0.082
    if subtitle.strip():
        ax.text(
            cx,
            y + h - 0.088,
            subtitle,
            fontsize=8.3,
            color="#475569",
            va="top",
            ha="center",
            multialignment="center",
            linespacing=1.22,
            zorder=1,
        )
        divider_y = y + h - 0.118
    ax.plot([x + 0.016, x + w - 0.016], [divider_y, divider_y], color=ec, linewidth=0.9, alpha=0.70, zorder=1)


def add_round_box(ax, x: float, y: float, w: float, h: float, title: str, body: str, fc: str, ec: str = "#334155") -> None:
    shadow = patches.FancyBboxPatch(
        (x + 0.006, y - 0.008),
        w,
        h,
        boxstyle="round,pad=0.016,rounding_size=0.03",
        linewidth=0.0,
        facecolor="#94a3b8",
        alpha=0.18,
        zorder=1,
    )
    ax.add_patch(shadow)
    patch = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.016,rounding_size=0.03",
        linewidth=1.25,
        facecolor=fc,
        edgecolor=ec,
        zorder=2,
    )
    ax.add_patch(patch)
    cx = x + w * 0.5
    ax.text(cx, y + h - 0.034, title, fontsize=10.2, fontweight="bold", color="#0f172a", va="top", ha="center", zorder=3)
    ax.text(
        cx,
        y + h - 0.078,
        body,
        fontsize=8.0,
        color="#1f2937",
        va="top",
        ha="center",
        multialignment="center",
        linespacing=1.26,
        zorder=3,
    )


def add_arrow(
    ax,
    start: tuple[float, float],
    end: tuple[float, float],
    text: str,
    color: str = "#475569",
    rad: float = 0.0,
    text_xy: tuple[float, float] | None = None,
    linestyle: str = "-",
) -> None:
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=13,
        linewidth=1.35,
        color=color,
        connectionstyle=f"arc3,rad={rad}",
        linestyle=linestyle,
        zorder=4,
    )
    ax.add_patch(arrow)
    if not text:
        return
    mx = (start[0] + end[0]) * 0.5 if text_xy is None else text_xy[0]
    my = (start[1] + end[1]) * 0.5 if text_xy is None else text_xy[1]
    ax.text(
        mx,
        my,
        text,
        fontsize=8.1,
        color=color,
        ha="center",
        va="center",
        bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor="#e2e8f0", alpha=0.95),
        zorder=5,
    )


def add_poly_arrow(
    ax,
    points: list[tuple[float, float]],
    text: str = "",
    color: str = "#475569",
    text_xy: tuple[float, float] | None = None,
    linewidth: float = 1.45,
    linestyle: str = "-",
) -> None:
    if len(points) < 2:
        return
    for start, end in zip(points[:-2], points[1:-1]):
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color=color,
            linewidth=linewidth,
            linestyle=linestyle,
            solid_capstyle="round",
            zorder=4,
        )
    arrow = FancyArrowPatch(
        points[-2],
        points[-1],
        arrowstyle="-|>",
        mutation_scale=13,
        linewidth=linewidth,
        color=color,
        linestyle=linestyle,
        zorder=4,
    )
    ax.add_patch(arrow)
    if not text:
        return
    if text_xy is None:
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        text_xy = (float(np.mean(xs)), float(np.mean(ys)))
    ax.text(
        text_xy[0],
        text_xy[1],
        text,
        fontsize=8.0,
        color=color,
        ha="center",
        va="center",
        bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor="#e2e8f0", alpha=0.95),
        zorder=5,
    )


def add_bidir_arrow(
    ax,
    start: tuple[float, float],
    end: tuple[float, float],
    text: str = "",
    color: str = "#475569",
    text_xy: tuple[float, float] | None = None,
    linewidth: float = 1.5,
    linestyle: str = "-",
) -> None:
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="<|-|>",
        mutation_scale=13,
        linewidth=linewidth,
        color=color,
        linestyle=linestyle,
        zorder=4,
    )
    ax.add_patch(arrow)
    if not text:
        return
    mx = (start[0] + end[0]) * 0.5 if text_xy is None else text_xy[0]
    my = (start[1] + end[1]) * 0.5 if text_xy is None else text_xy[1]
    ax.text(
        mx,
        my,
        text,
        fontsize=8.2,
        color=color,
        ha="center",
        va="center",
        bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor="#e2e8f0", alpha=0.96),
        zorder=5,
    )


def add_clean_box(
    ax,
    x: float,
    y: float,
    w: float,
    h: float,
    title: str,
    body: str = "",
    fc: str = "#ffffff",
    ec: str = "#334155",
) -> None:
    patch = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.012,rounding_size=0.01",
        linewidth=1.3,
        facecolor=fc,
        edgecolor=ec,
        zorder=2,
    )
    ax.add_patch(patch)
    cx = x + w * 0.5
    ax.text(cx, y + h * 0.61, title, fontsize=12.4, fontweight="bold", color="#111827", ha="center", va="center", zorder=3)
    if body.strip():
        ax.text(
            cx,
            y + h * 0.28,
            body,
            fontsize=8.6,
            color="#374151",
            ha="center",
            va="center",
            multialignment="center",
            linespacing=1.16,
            zorder=3,
        )


def add_protocol_hex(
    ax,
    x: float,
    y: float,
    w: float,
    h: float,
    top: str,
    protocol: str,
    bottom: str,
    protocol_color: str = "#dc2626",
    fc: str = "#ffffff",
    ec: str = "#374151",
) -> None:
    dx = min(w * 0.12, 0.03)
    pts = np.array(
        [
            [x + dx, y],
            [x + w - dx, y],
            [x + w, y + h * 0.5],
            [x + w - dx, y + h],
            [x + dx, y + h],
            [x, y + h * 0.5],
        ],
        dtype=float,
    )
    patch = patches.Polygon(pts, closed=True, facecolor=fc, edgecolor=ec, linewidth=1.35, zorder=2)
    ax.add_patch(patch)
    cx = x + w * 0.5
    if top.strip():
        ax.text(cx, y + h * 0.70, top, fontsize=9.8, color="#1f2937", ha="center", va="center", zorder=3)
    ax.text(cx, y + h * 0.50, protocol, fontsize=11.6, fontweight="bold", color=protocol_color, ha="center", va="center", zorder=3)
    if bottom.strip():
        ax.text(
            cx,
            y + h * 0.27,
            bottom,
            fontsize=9.2,
            color="#374151",
            ha="center",
            va="center",
            multialignment="center",
            linespacing=1.15,
            zorder=3,
        )


def add_junction(ax, x: float, y: float, color: str = "#475569", radius: float = 0.008) -> None:
    node = patches.Circle((x, y), radius=radius, facecolor=color, edgecolor="white", linewidth=0.9, zorder=6)
    ax.add_patch(node)


def add_flow_legend(ax, items: list[tuple[str, str]], x: float, y: float, dx: float = 0.13) -> None:
    for idx, (label, color) in enumerate(items):
        x0 = x + idx * dx
        ax.plot([x0, x0 + 0.03], [y, y], color=color, linewidth=2.2, solid_capstyle="round", zorder=6)
        ax.text(x0 + 0.036, y, label, fontsize=8.6, color="#334155", va="center", ha="left", zorder=6)


def plot_system_block_diagram() -> None:
    fig, ax = plt.subplots(figsize=(15.0, 7.8), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    add_clean_box(ax, 0.04, 0.58, 0.11, 0.16, "QGC", fc="#ffffff")
    add_protocol_hex(
        ax,
        0.18,
        0.54,
        0.20,
        0.20,
        "조종 입력",
        "MAVLink UDP",
        "상태 / heartbeat / telemetry",
    )
    add_clean_box(ax, 0.43, 0.58, 0.11, 0.16, "ArduSub", fc="#ffffff", ec="#2563eb")
    add_protocol_hex(
        ax,
        0.59,
        0.54,
        0.22,
        0.20,
        "servo / thruster command",
        "JSON SITL UDP",
        "sim sensor values\n(depth, rangefinder,\nattitude / NED state)",
    )
    add_clean_box(ax, 0.86, 0.58, 0.11, 0.16, "MuJoCo", fc="#ffffff")

    add_clean_box(ax, 0.43, 0.22, 0.11, 0.16, "MAVROS", fc="#ffffff", ec="#f59e0b")
    add_clean_box(ax, 0.86, 0.22, 0.11, 0.16, "ROS2 rospkg", fc="#ffffff")

    add_bidir_arrow(ax, (0.15, 0.66), (0.18, 0.66), "")
    add_bidir_arrow(ax, (0.38, 0.66), (0.43, 0.66), "")
    add_bidir_arrow(ax, (0.54, 0.66), (0.59, 0.66), "")
    add_bidir_arrow(ax, (0.81, 0.66), (0.86, 0.66), "")

    add_bidir_arrow(
        ax,
        (0.485, 0.58),
        (0.485, 0.38),
        "MAVLink UDP 14551",
        color="#ea580c",
        text_xy=(0.58, 0.48),
    )
    add_bidir_arrow(
        ax,
        (0.54, 0.30),
        (0.86, 0.30),
        "ROS2 topics / services\n/mavros/rc/override, /set_mode,\n/mavros/local_position/odom",
        color="#7c3aed",
        text_xy=(0.70, 0.40),
    )

    ax.text(0.50, 0.96, "UUV MuJoCo v2.2 정석 호환 시스템 구조", ha="center", va="top", fontsize=16.2, fontweight="bold")
    fig.savefig(FIG_DIR / "uuv_v22_latest_system_block_diagram.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_comm_diagram() -> None:
    fig, ax = plt.subplots(figsize=(15.2, 8.0), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    add_clean_box(ax, 0.05, 0.64, 0.11, 0.14, "QGC", fc="#ffffff")
    add_protocol_hex(
        ax,
        0.21,
        0.61,
        0.22,
        0.18,
        "joystick / mode / arm",
        "MAVLink UDP",
        "state / heartbeat /\ntelemetry",
    )
    add_clean_box(ax, 0.47, 0.64, 0.12, 0.14, "ArduSub", fc="#ffffff", ec="#2563eb")
    add_protocol_hex(
        ax,
        0.64,
        0.61,
        0.24,
        0.18,
        "servo / thruster command",
        "JSON SITL UDP",
        "depth / rangefinder /\nattitude / NED state",
    )
    add_clean_box(ax, 0.89, 0.64, 0.08, 0.14, "MuJoCo", fc="#ffffff")

    add_clean_box(ax, 0.05, 0.22, 0.12, 0.14, "ROS2 rospkg", fc="#ffffff")
    add_protocol_hex(
        ax,
        0.22,
        0.19,
        0.22,
        0.18,
        "arming / set_mode /\nrc_override / setpoint",
        "ROS2 topics / services",
        "state / odom / imu /\npressure / battery",
        protocol_color="#7c3aed",
    )
    add_clean_box(ax, 0.48, 0.22, 0.11, 0.14, "MAVROS", fc="#ffffff", ec="#f59e0b")
    add_protocol_hex(
        ax,
        0.64,
        0.19,
        0.20,
        0.18,
        "MAVLink bridge",
        "UDP 14551",
        "ArduSub state + command",
        protocol_color="#ea580c",
    )
    add_clean_box(ax, 0.87, 0.22, 0.11, 0.14, "ArduSub", fc="#ffffff", ec="#2563eb")

    add_bidir_arrow(ax, (0.16, 0.71), (0.21, 0.71), "")
    add_bidir_arrow(ax, (0.43, 0.71), (0.47, 0.71), "")
    add_bidir_arrow(ax, (0.59, 0.71), (0.64, 0.71), "")
    add_bidir_arrow(ax, (0.88, 0.71), (0.89, 0.71), "")

    add_bidir_arrow(ax, (0.17, 0.29), (0.22, 0.29), "")
    add_bidir_arrow(ax, (0.44, 0.29), (0.48, 0.29), "")
    add_bidir_arrow(ax, (0.59, 0.29), (0.64, 0.29), "")
    add_bidir_arrow(ax, (0.84, 0.29), (0.87, 0.29), "")

    ax.text(0.50, 0.96, "정석 호환 기준 제어 / 통신 경로", ha="center", va="top", fontsize=16.0, fontweight="bold")
    ax.text(0.50, 0.91, "위: QGC-ArduSub-MuJoCo 시뮬레이션 루프   아래: ROS2 rospkg-MAVROS-ArduSub ROS 제어 루프", ha="center", va="top", fontsize=10.3, color="#475569")
    fig.savefig(FIG_DIR / "uuv_v22_latest_sitl_comm_diagram.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_physics_pipeline() -> None:
    fig, ax = plt.subplots(figsize=(15.2, 8.5), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    add_lane(ax, 0.04, 0.14, 0.56, 0.76, "Python Runtime: Custom Logic and Force Synthesis", "#eff6ff")
    add_lane(ax, 0.64, 0.14, 0.32, 0.76, "MuJoCo Engine: Built-in Dynamics and Sensors", "#ecfdf5")

    add_round_box(ax, 0.07, 0.70, 0.21, 0.14, "1. Command ingestion", "SERVO_OUTPUT_RAW / JSON servo\n/mavros/rc/override / /cmd_vel", "#dbeafe")
    add_round_box(ax, 0.07, 0.49, 0.21, 0.14, "2. Thruster shaping", "Deadzone\nFirst-order lag\nPWM-force curve\nReverse asymmetry", "#dbeafe")
    add_round_box(ax, 0.07, 0.28, 0.21, 0.14, "3. Mass / inertia synthesis", "body_components -> CoM\nParallel-axis inertia\nbody_inertia_scale_xyz", "#dbeafe")

    add_round_box(ax, 0.33, 0.70, 0.23, 0.14, "4. Hydrostatics", "Distributed buoyancy points\nsubmerged_fraction()\nCoB lever-arm torque", "#fef3c7")
    add_round_box(ax, 0.33, 0.49, 0.23, 0.14, "5A. Legacy branch", "Added mass\nC_A(nu) Coriolis\nlinear / quadratic damping", "#fce7f3")
    add_round_box(ax, 0.33, 0.28, 0.23, 0.14, "5B. Current branch", "tank_current_scene.xml\nfluidshape=\"ellipsoid\"\nfluidcoef-driven drag", "#dcfce7")

    add_round_box(ax, 0.68, 0.70, 0.24, 0.14, "6. Force application", "data.ctrl\n+ data.xfrc_applied\n(world-frame wrench)", "#dcfce7")
    add_round_box(ax, 0.68, 0.49, 0.24, 0.14, "7. mj_step()", "Rigid-body integration\nConstraint / contact\nBuilt-in fluid evaluation", "#dcfce7")
    add_round_box(ax, 0.68, 0.28, 0.24, 0.14, "8. Sensor export", "IMU / DVL / depth / odom\nJSON SITL state\nROS2 publish surface", "#dcfce7")

    add_arrow(ax, (0.28, 0.77), (0.33, 0.77), "")
    add_arrow(ax, (0.28, 0.56), (0.33, 0.56), "")
    add_arrow(ax, (0.28, 0.35), (0.33, 0.35), "")
    add_arrow(ax, (0.56, 0.56), (0.68, 0.77), "legacy only", text_xy=(0.62, 0.68))
    add_arrow(ax, (0.56, 0.35), (0.68, 0.77), "current only", text_xy=(0.62, 0.44))
    add_arrow(ax, (0.80, 0.70), (0.80, 0.63), "")
    add_arrow(ax, (0.80, 0.49), (0.80, 0.42), "")

    callout = patches.FancyBboxPatch(
        (0.06, 0.08),
        0.88,
        0.18,
        boxstyle="round,pad=0.02,rounding_size=0.03",
        facecolor="#f8fafc",
        edgecolor="#cbd5e1",
        linewidth=1.0,
    )
    ax.add_patch(callout)
    ax.text(0.08, 0.215, "Core equations used in the report", fontsize=10.5, fontweight="bold", color="#0f172a", va="top")
    ax.text(
        0.08,
        0.185,
        r"$F_B=\rho g V_{sub}$, "
        r"$\tau_B=\sum_i (r_i-r_{CoM}) \times F_{B_i}$, "
        r"$\nu=[u,v,w,p,q,r]^T$, "
        r"$\tau_h=-M_A\dot{\nu}-C_A(\nu)\nu-D_1\nu-D_2|\nu|\nu$",
        fontsize=10.2,
        color="#1f2937",
        va="top",
    )
    ax.text(
        0.08,
        0.125,
        "Current v2.2의 핵심은 legacy branch를 없애고 5B를 중심으로 옮긴 뒤, 매우 작은 runtime inertia와 더 큰 CoB torque / drag를 함께 적용한 점이다.",
        fontsize=9.0,
        color="#334155",
        va="top",
    )

    ax.text(0.50, 0.97, "v2.2 물리엔진 모델링 파이프라인", ha="center", va="top", fontsize=16, fontweight="bold")
    ax.text(
        0.50,
        0.935,
        "어떤 항이 Python에서 계산되고 어떤 항이 MuJoCo built-in으로 처리되는지 분리해서 표현했다. legacy와 current의 분기점도 같은 도식 안에 넣었다.",
        ha="center",
        va="top",
        fontsize=10,
        color="#475569",
    )
    fig.savefig(FIG_DIR / "uuv_v22_latest_physics_pipeline.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_profile_delta(profiles: dict, legacy_meas: dict, current_meas: dict) -> None:
    legacy = profiles["legacy"]
    current = profiles["current"]

    fig, axes = plt.subplots(2, 2, figsize=(13.6, 9.5), dpi=220, constrained_layout=True)

    knobs = ["thruster_force_max", "linear_drag", "angular_drag", "cob_torque_scale"]
    labels = ["Thrust max [N]", "Linear drag", "Angular drag", "CoB torque scale"]
    lx = [float(legacy[k]) for k in knobs]
    cx = [float(current[k]) for k in knobs]
    x = np.arange(len(labels))
    width = 0.35
    axes[0, 0].bar(x - width / 2, lx, width, color=MODEL_COLORS["legacy"], label="Legacy")
    axes[0, 0].bar(x + width / 2, cx, width, color=MODEL_COLORS["current"], label="Current")
    axes[0, 0].set_xticks(x, labels, rotation=15, ha="right")
    axes[0, 0].set_title("핵심 tuning knob 변화")
    axes[0, 0].grid(True, axis="y", alpha=0.25)
    axes[0, 0].legend(frameon=False)

    inertia_vals = [
        float(np.mean(np.asarray(legacy.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]), dtype=float))),
        float(np.mean(np.asarray(current.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]), dtype=float))),
    ]
    axes[0, 1].bar(["Legacy", "Current"], inertia_vals, color=[MODEL_COLORS["legacy"], MODEL_COLORS["current"]])
    axes[0, 1].set_yscale("log")
    axes[0, 1].set_title("Runtime body inertia scale")
    axes[0, 1].grid(True, axis="y", alpha=0.25)

    offsets = {
        "CoB x offset [m]": (float(legacy.get("cob_x_offset", 0.0)), float(current.get("cob_x_offset", 0.0))),
        "CoB z offset [m]": (float(legacy.get("cob_z_offset", 0.0)), float(current.get("cob_z_offset", 0.0))),
        "Buoyancy scale": (float(legacy.get("buoyancy_scale", 1.0)), float(current.get("buoyancy_scale", 1.0))),
        "Voltage [V]": (float(legacy.get("thruster_voltage", 0.0)), float(current.get("thruster_voltage", 0.0))),
    }
    x2 = np.arange(len(offsets))
    axes[1, 0].bar(x2 - width / 2, [v[0] for v in offsets.values()], width, color=MODEL_COLORS["legacy"])
    axes[1, 0].bar(x2 + width / 2, [v[1] for v in offsets.values()], width, color=MODEL_COLORS["current"])
    axes[1, 0].set_xticks(x2, list(offsets.keys()), rotation=15, ha="right")
    axes[1, 0].set_title("부력 / 오프셋 / 전압 파라미터")
    axes[1, 0].grid(True, axis="y", alpha=0.25)

    def avg_metric(measurement: dict, key: str) -> float:
        vals = [float(measurement["modes"][mode]["summary"][key]) for mode in MODE_ORDER]
        return float(sum(vals) / len(vals))

    actual_old = [
        avg_metric(legacy_meas, "trajectory_path_length_m"),
        avg_metric(legacy_meas, "horizontal_distance_m"),
        avg_metric(legacy_meas, "max_abs_pitch_deg"),
        avg_metric(legacy_meas, "max_abs_roll_deg"),
    ]
    actual_new = [
        avg_metric(current_meas, "trajectory_path_length_m"),
        avg_metric(current_meas, "horizontal_distance_m"),
        avg_metric(current_meas, "max_abs_pitch_deg"),
        avg_metric(current_meas, "max_abs_roll_deg"),
    ]
    actual_labels = ["Path [m]", "Horizontal [m]", "Max |pitch| [deg]", "Max |roll| [deg]"]
    x3 = np.arange(len(actual_labels))
    axes[1, 1].bar(x3 - width / 2, actual_old, width, color=MODEL_COLORS["legacy"])
    axes[1, 1].bar(x3 + width / 2, actual_new, width, color=MODEL_COLORS["current"])
    axes[1, 1].set_xticks(x3, actual_labels, rotation=15, ha="right")
    axes[1, 1].set_title("실측 outcome 변화")
    axes[1, 1].grid(True, axis="y", alpha=0.25)

    fig.suptitle("legacy 대비 current v2.2 tuning 변화와 실제 결과", fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / "uuv_v22_latest_profile_delta.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_ellipsoid_surface(ax, center: np.ndarray, radii: np.ndarray, color: str, alpha: float = 0.18) -> None:
    u = np.linspace(0.0, 2.0 * math.pi, 32)
    v = np.linspace(0.0, math.pi, 18)
    x = center[0] + radii[0] * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radii[1] * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radii[2] * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(y, x, z, color=color, alpha=alpha, linewidth=0.0, shade=False)


def plot_ellipsoid_layout(scene: dict, profiles: dict) -> None:
    current = profiles["current"]
    summary = composite_summary(current)

    fig = plt.figure(figsize=(12.0, 9.0), dpi=220, constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")

    for geom in scene["fluid_geoms"]:
        plot_ellipsoid_surface(ax, geom["pos"], geom["size"], "#60a5fa", alpha=0.12)
        pos = geom["pos"]
        ax.text(pos[1], pos[0], pos[2] + 0.045, geom["name"].replace("fluid_", ""), fontsize=7.8, color="#1d4ed8")

    for comp in current["body_components"]:
        pos = np.asarray(comp["mass_pos"], dtype=float)
        ax.scatter(pos[1], pos[0], pos[2], color="#ef4444", s=28)

    for point in current["buoyancy_points"]:
        pos = np.asarray(point["pos"], dtype=float)
        ax.scatter(pos[1], pos[0], pos[2], color="#16a34a", s=26)

    com = summary["com"]
    cob = summary["buoy_center"]
    ax.scatter(com[1], com[0], com[2], color="black", s=56, marker="x")
    ax.scatter(cob[1], cob[0], cob[2], color="#059669", s=58, marker="^")
    ax.text(com[1], com[0], com[2] + 0.028, "CoM", fontsize=8.6, color="black")
    ax.text(cob[1], cob[0], cob[2] + 0.028, "CoB", fontsize=8.6, color="#065f46")

    for name, actuator in scene["actuators"].items():
        site = actuator["site"]
        if not site or site not in scene["sites"]:
            continue
        pos = scene["sites"][site]
        vec = actuator["gear"]
        norm = np.linalg.norm(vec)
        if norm <= 1e-9:
            continue
        vec = 0.08 * (vec / norm)
        ax.quiver(pos[1], pos[0], pos[2], vec[1], vec[0], vec[2], color="#334155", linewidth=0.9, arrow_length_ratio=0.16)

    ax.set_xlabel("Y [m]")
    ax.set_ylabel("X [m]")
    ax.set_zlabel("Z [m]")
    ax.view_init(elev=24, azim=-58)
    try:
        ax.set_box_aspect((0.8, 1.2, 0.5))
    except Exception:
        pass
    ax.set_title("Current scene: drag proxy / mass / buoyancy layout", pad=12)
    legend_handles = [
        Line2D([0], [0], color="#60a5fa", linewidth=8, label="Ellipsoid drag proxy"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#ef4444", markersize=7, label="Body component mass"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#16a34a", markersize=7, label="Buoyancy point"),
        Line2D([0], [0], marker="x", color="black", markersize=7, label="CoM"),
        Line2D([0], [0], marker="^", color="#059669", markersize=7, label="CoB"),
    ]
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(0.01, 0.99), frameon=False)
    fig.savefig(FIG_DIR / "uuv_v22_latest_ellipsoid_layout.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_mode_xy_compare(legacy: dict, current: dict) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 10.4), dpi=220, constrained_layout=True)
    for ax, mode_key in zip(axes.flat, MODE_ORDER):
        lx, ly, _ = mode_arrays(legacy, mode_key)
        cx, cy, _ = mode_arrays(current, mode_key)
        ax.plot(ly, lx, color=MODEL_COLORS["legacy"], linewidth=2.1, label="Legacy")
        ax.plot(cy, cx, color=MODEL_COLORS["current"], linewidth=2.1, label="Current")
        ax.scatter([ly[0], cy[0]], [lx[0], cx[0]], color=["#a855f7", "#0f766e"], s=20, zorder=3)
        ax.set_title(MODE_LABELS[mode_key])
        ax.set_xlabel("Y position [m]")
        ax.set_ylabel("X position [m]")
        ax.grid(True, alpha=0.25)
        ax.axis("equal")
    handles = [
        Line2D([0], [0], color=MODEL_COLORS["legacy"], linewidth=2.2, label="Legacy"),
        Line2D([0], [0], color=MODEL_COLORS["current"], linewidth=2.2, label="Current v2.2"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.01))
    fig.suptitle("실측 rosbag 기반 mode-path XY 비교", y=1.02, fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / "uuv_v22_latest_actual_mode_xy_compare.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def smooth_display_series(values: np.ndarray, window: int) -> np.ndarray:
    if values.size < 3:
        return values.copy()
    win = int(max(3, window))
    if win % 2 == 0:
        win += 1
    if values.size <= win:
        return values.copy()
    kernel = np.ones(win, dtype=float) / float(win)
    pad = win // 2
    padded = np.pad(values, pad, mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def plot_mode_3d_compare(legacy: dict, current: dict) -> None:
    fig = plt.figure(figsize=(14.2, 11.0), dpi=220, constrained_layout=True)
    axes = [
        fig.add_subplot(2, 2, 1, projection="3d"),
        fig.add_subplot(2, 2, 2, projection="3d"),
        fig.add_subplot(2, 2, 3, projection="3d"),
        fig.add_subplot(2, 2, 4, projection="3d"),
    ]

    def collect_limits(measurement: dict) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        for mode in measurement["modes"].values():
            pts = mode["summary"]["trajectory_xyz_depth"]
            xs.extend(float(p["x"]) for p in pts)
            ys.extend(float(p["y"]) for p in pts)
            zs.extend(float(p["depth"]) for p in pts)
        x_pad = max((max(xs) - min(xs)) * 0.10, 0.006)
        y_pad = max((max(ys) - min(ys)) * 0.10, 0.006)
        z_pad = max((max(zs) - min(zs)) * 0.16, 0.0015)
        return (
            (min(xs) - x_pad, max(xs) + x_pad),
            (min(ys) - y_pad, max(ys) + y_pad),
            (min(zs) - z_pad, max(zs) + z_pad),
        )

    legacy_lim = collect_limits(legacy)
    current_lim = collect_limits(current)
    global_lim = collect_limits(
        {
            "modes": {
                **legacy["modes"],
                **{f"current_{key}": value for key, value in current["modes"].items()},
            }
        }
    )

    def draw_panel(
        ax,
        measurement: dict,
        title: str,
        limits: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
        subtitle: str,
    ) -> None:
        x_lim, y_lim, z_lim = limits
        z_floor = z_lim[0]
        for mode_key in MODE_ORDER:
            x, y, depth = mode_arrays(measurement, mode_key)
            color = MODE_COLORS[mode_key]
            x_s = smooth_display_series(x, 5)
            y_s = smooth_display_series(y, 5)
            depth_s = smooth_display_series(depth, 11)
            ax.plot(x, y, depth, color=color, linewidth=0.65, alpha=0.16)
            ax.plot(x_s, y_s, depth_s, color=color, linewidth=2.2, alpha=0.98)
            ax.plot(x_s, y_s, np.full_like(depth_s, z_floor), color=color, linewidth=0.8, alpha=0.16)
            ax.scatter([x[0]], [y[0]], [depth[0]], color=color, s=18, marker="o", depthshade=False)
            ax.scatter([x[-1]], [y[-1]], [depth[-1]], color=color, s=22, marker="X", depthshade=False)
        ax.set_title(f"{title}\n{subtitle}", pad=10)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Depth [m]")
        ax.set_xlim(x_lim)
        ax.set_ylim(y_lim)
        ax.set_zlim(z_lim[1], z_lim[0])
        ax.view_init(elev=23, azim=-56)
        ax.grid(True, alpha=0.20)
        span_x = x_lim[1] - x_lim[0]
        span_y = y_lim[1] - y_lim[0]
        span_z = max((z_lim[1] - z_lim[0]) * 1.1, 0.015)
        try:
            ax.set_box_aspect((span_x, span_y, span_z))
        except Exception:
            pass
        if hasattr(ax, "set_proj_type"):
            ax.set_proj_type("ortho")
        ax.text2D(
            0.03,
            0.97,
            "bold = smoothed display path   start = o   end = X",
            transform=ax.transAxes,
            fontsize=8.0,
            color="#475569",
            va="top",
            bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor="#e2e8f0", alpha=0.92),
        )

    draw_panel(axes[0], legacy, "Legacy", global_lim, "same global scale")
    draw_panel(axes[1], current, "Current v2.2", global_lim, "same global scale")
    draw_panel(axes[2], legacy, "Legacy", legacy_lim, "local zoom")
    draw_panel(axes[3], current, "Current v2.2", current_lim, "local zoom")

    handles = [Line2D([0], [0], color=MODE_COLORS[m], linewidth=2.2, label=MODE_LABELS[m]) for m in MODE_ORDER]
    fig.legend(handles=handles, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.01))
    fig.suptitle("실측 rosbag 기반 3D trajectory 비교", y=1.025, fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / "uuv_v22_latest_actual_mode_3d_compare.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_metric_compare(legacy: dict, current: dict) -> None:
    labels = [MODE_LABELS[m] for m in MODE_ORDER]
    x = np.arange(len(MODE_ORDER))
    width = 0.36

    def vals(measurement: dict, key: str) -> list[float]:
        out = []
        for mode_key in MODE_ORDER:
            mode = measurement["modes"][mode_key]
            if key == "path":
                out.append(float(mode["summary"]["trajectory_path_length_m"]))
            elif key == "horiz":
                out.append(float(mode["summary"]["horizontal_distance_m"]))
            elif key == "pitch":
                out.append(float(mode["summary"]["max_abs_pitch_deg"]))
            elif key == "roll":
                out.append(float(mode["summary"]["max_abs_roll_deg"]))
        return out

    metrics = {
        "Trajectory path [m]": vals(legacy, "path"),
        "Horizontal distance [m]": vals(legacy, "horiz"),
        "Max |pitch| [deg]": vals(legacy, "pitch"),
        "Max |roll| [deg]": vals(legacy, "roll"),
    }
    metrics_new = {
        "Trajectory path [m]": vals(current, "path"),
        "Horizontal distance [m]": vals(current, "horiz"),
        "Max |pitch| [deg]": vals(current, "pitch"),
        "Max |roll| [deg]": vals(current, "roll"),
    }

    fig, axes = plt.subplots(2, 2, figsize=(13.2, 9.2), dpi=220, constrained_layout=True)
    for ax, title in zip(axes.flat, metrics.keys()):
        legacy_vals = metrics[title]
        current_vals = metrics_new[title]
        ax.bar(x - width / 2, legacy_vals, width=width, color=MODEL_COLORS["legacy"], label="Legacy")
        ax.bar(x + width / 2, current_vals, width=width, color=MODEL_COLORS["current"], label="Current")
        ax.set_title(title)
        ax.set_xticks(x, labels)
        ax.grid(True, axis="y", alpha=0.25)
    handles = [
        Line2D([0], [0], color=MODEL_COLORS["legacy"], linewidth=8, label="Legacy"),
        Line2D([0], [0], color=MODEL_COLORS["current"], linewidth=8, label="Current v2.2"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.01))
    fig.suptitle("실측 rosbag 정량 지표 비교", y=1.02, fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / "uuv_v22_latest_actual_metric_compare.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def copy_step_plots() -> None:
    if CURRENT_STEP_PLOT.exists():
        shutil.copy2(CURRENT_STEP_PLOT, FIG_DIR / "uuv_v22_latest_actual_step_responses.png")
    if CURRENT_MODE_PLOT.exists():
        shutil.copy2(CURRENT_MODE_PLOT, FIG_DIR / "uuv_v22_latest_actual_mode_response.png")


def write_summary_json(legacy: dict, current: dict, profiles: dict, scene: dict, step_summary: dict, sitl_params: dict) -> None:
    def average_metric(measurement: dict, key: str) -> float:
        vals = [float(measurement["modes"][mode]["summary"][key]) for mode in MODE_ORDER]
        return float(sum(vals) / len(vals))

    legacy_summary = composite_summary(profiles["legacy"])
    current_summary = composite_summary(profiles["current"])
    neutral_volume = float(current_summary["mass_total"] / 1000.0)
    proxy_volume = float(sum(item["volume_m3"] for item in scene["fluid_geoms"]))
    scene_geoms = [
        {
            "name": item["name"],
            "pos": item["pos"].tolist(),
            "size": item["size"].tolist(),
            "fluidcoef": item["fluidcoef"].tolist(),
            "volume_m3": item["volume_m3"],
        }
        for item in scene["fluid_geoms"]
    ]

    payload = {
        "measurement_date": "2026-04-06",
        "actual_bag_paths": {
            "legacy_mode_path": str(LEGACY_MEAS_PATH),
            "current_mode_path": str(CURRENT_MEAS_PATH),
            "current_step_summary": str(CURRENT_STEP_SUMMARY_PATH),
        },
        "averages": {
            "legacy": {
                "path_m": average_metric(legacy, "trajectory_path_length_m"),
                "horizontal_m": average_metric(legacy, "horizontal_distance_m"),
                "pitch_deg": average_metric(legacy, "max_abs_pitch_deg"),
                "roll_deg": average_metric(legacy, "max_abs_roll_deg"),
            },
            "current": {
                "path_m": average_metric(current, "trajectory_path_length_m"),
                "horizontal_m": average_metric(current, "horizontal_distance_m"),
                "pitch_deg": average_metric(current, "max_abs_pitch_deg"),
                "roll_deg": average_metric(current, "max_abs_roll_deg"),
            },
        },
        "reductions_percent": {},
        "scene": {
            "fluid_geoms": scene_geoms,
            "proxy_volume_total_m3": proxy_volume,
            "neutral_volume_m3": neutral_volume,
        },
        "runtime_mass_properties": {
            "legacy": {
                "mass_kg": legacy_summary["mass_total"],
                "com_m": legacy_summary["com"].tolist(),
                "cob_m": legacy_summary["buoy_center"].tolist(),
                "inertia_raw_kgm2": legacy_summary["inertia_raw"].tolist(),
                "inertia_scaled_kgm2": legacy_summary["inertia_scaled"].tolist(),
            },
            "current": {
                "mass_kg": current_summary["mass_total"],
                "com_m": current_summary["com"].tolist(),
                "cob_m": current_summary["buoy_center"].tolist(),
                "inertia_raw_kgm2": current_summary["inertia_raw"].tolist(),
                "inertia_scaled_kgm2": current_summary["inertia_scaled"].tolist(),
            },
        },
        "sitl_params": {key: sitl_params[key] for key in ("AHRS_EKF_TYPE", "GPS1_TYPE", "GPS2_TYPE", "RNGFND1_TYPE", "THR_DZ", "PILOT_SPEED_UP", "PILOT_SPEED_DN", "PILOT_ACCEL_Z")},
        "current_step_summary": step_summary.get("summary", {}),
    }

    for key in ("path_m", "horizontal_m", "pitch_deg", "roll_deg"):
        payload["reductions_percent"][key] = percent_reduction(payload["averages"]["legacy"][key], payload["averages"]["current"][key])

    SUMMARY_JSON_PATH.write_text(json.dumps(payload, indent=2, ensure_ascii=False))


def main() -> None:
    ensure_dirs()
    profiles = load_profiles()
    legacy_meas, current_meas = load_measurements()
    step_summary = load_step_summary()
    scene = parse_scene(CURRENT_SCENE)
    sitl_params = parse_sitl_params(START_SCRIPT)

    plot_system_block_diagram()
    plot_comm_diagram()
    plot_physics_pipeline()
    plot_profile_delta(profiles, legacy_meas, current_meas)
    plot_ellipsoid_layout(scene, profiles)
    plot_mode_xy_compare(legacy_meas, current_meas)
    plot_mode_3d_compare(legacy_meas, current_meas)
    plot_metric_compare(legacy_meas, current_meas)
    copy_step_plots()
    write_summary_json(legacy_meas, current_meas, profiles, scene, step_summary, sitl_params)


if __name__ == "__main__":
    main()

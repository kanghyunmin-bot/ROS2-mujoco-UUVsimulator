#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


ROOT = Path(__file__).resolve().parents[2]
DOCSRC_DIR = Path(__file__).resolve().parent
FIG_DIR = DOCSRC_DIR / "figures_v30"
SIM_DIR = ROOT / "uuv_mujoco" / "v2.2"
CONFIG_PATH = SIM_DIR / "config" / "sim_profiles.json"
CURRENT_SCENE = SIM_DIR / "scenes" / "tank_current_scene.xml"
START_SCRIPT = SIM_DIR / "start_ardusub_sitl_mj311.sh"
CURRENT_MEAS_PATH = DOCSRC_DIR / "measurement_current_mode_path_latest_v30.json"
LEGACY_MEAS_PATH = DOCSRC_DIR / "measurement_legacy_mode_path_latest_v30.json"
CURRENT_STEP_SUMMARY_PATH = DOCSRC_DIR / "measurement_summary_current_latest_v30.json"
SUMMARY_JSON_PATH = DOCSRC_DIR / "uuv_v30_report_metrics.json"

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
        fluid_geoms.append(
            {
                "name": name,
                "pos": np.array([float(v) for v in geom.attrib.get("pos", "0 0 0").split()], dtype=float),
                "size": np.array([float(v) for v in geom.attrib.get("size", "0 0 0").split()], dtype=float),
                "fluidcoef": np.array([float(v) for v in geom.attrib.get("fluidcoef", "0 0 0 0 0").split()], dtype=float),
            }
        )
    return {"sites": sites, "actuators": actuators, "fluid_geoms": fluid_geoms}


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


def mode_arrays(measurement: dict, mode_key: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points = measurement["modes"][mode_key]["summary"]["trajectory_xyz_depth"]
    x = np.array([float(p["x"]) for p in points], dtype=float)
    y = np.array([float(p["y"]) for p in points], dtype=float)
    depth = np.array([float(p["depth"]) for p in points], dtype=float)
    return x, y, depth


def percent_reduction(old: float, new: float) -> float:
    if abs(old) <= 1e-12:
        return 0.0
    return (1.0 - (new / old)) * 100.0


def wrapped(text: str, width: int) -> str:
    return textwrap.fill(text, width=width, break_long_words=False)


def add_round_box(ax, x: float, y: float, w: float, h: float, title: str, body: str, fc: str, ec: str = "#334155") -> None:
    patch = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.018,rounding_size=0.03",
        linewidth=1.4,
        facecolor=fc,
        edgecolor=ec,
    )
    ax.add_patch(patch)
    ax.text(x + 0.02, y + h - 0.05, title, fontsize=11.2, fontweight="bold", color="#0f172a", va="top")
    ax.text(x + 0.02, y + h - 0.11, body, fontsize=8.8, color="#1f2937", va="top", linespacing=1.35)


def add_arrow(
    ax,
    start: tuple[float, float],
    end: tuple[float, float],
    text: str,
    color: str = "#475569",
    rad: float = 0.0,
    text_xy: tuple[float, float] | None = None,
) -> None:
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=14,
        linewidth=1.4,
        color=color,
        connectionstyle=f"arc3,rad={rad}",
    )
    ax.add_patch(arrow)
    mx = (start[0] + end[0]) * 0.5 if text_xy is None else text_xy[0]
    my = (start[1] + end[1]) * 0.5 if text_xy is None else text_xy[1]
    if text:
        ax.text(
            mx,
            my + (0.02 if abs(rad) < 1e-6 else 0.05 * np.sign(rad)),
            text,
            fontsize=8.2,
            color=color,
            ha="center",
            va="center",
            bbox=dict(boxstyle="round,pad=0.16", facecolor="white", edgecolor="none", alpha=0.85),
        )


def plot_system_block_diagram() -> None:
    fig, ax = plt.subplots(figsize=(14.2, 8.6), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    add_round_box(ax, 0.04, 0.66, 0.18, 0.22, "QGroundControl", wrapped("조이스틱 / 모드 전환 / 시각화\nUDP 14550", 20), "#ede9fe")
    add_round_box(ax, 0.28, 0.67, 0.18, 0.20, "QGC MAVLink Proxy", wrapped("MANUAL_CONTROL axis hold\nUDP 14552", 20), "#dbeafe")
    add_round_box(ax, 0.52, 0.61, 0.20, 0.30, "ArduSub SITL", wrapped("제어기 / 모드 로직 / 자세·깊이 hold\nserial1=14552\nserial2=14551\nJSON SITL 9002/9003", 20), "#fee2e2")
    add_round_box(ax, 0.78, 0.60, 0.18, 0.32, "MuJoCo v3.0 Runtime", wrapped("run_urdf_full.py\nros2_bridge.py\nsitl_transport.py\nellipsoid fluid + runtime 보정", 20), "#dcfce7")

    add_round_box(ax, 0.05, 0.32, 0.18, 0.22, "ROS 패키지", wrapped("mavros_node\njoy2mavros\nodom2mavros\nvfr2atm_pressure", 18), "#fef3c7")
    add_round_box(ax, 0.30, 0.30, 0.18, 0.24, "uuv_control_gui.py", wrapped("ROS2 GUI\n/cmd_vel\n/mavros/* service", 18), "#fce7f3")
    add_round_box(ax, 0.55, 0.28, 0.18, 0.24, "ROS2 Surface", wrapped("/mavros/state\n/mavros/local_position/*\n/dvl/*\n/depth\n/imu/data", 18), "#e0f2fe")
    add_round_box(ax, 0.79, 0.28, 0.16, 0.24, "측정/문서", wrapped("rosbag2\nanalyze_mode_path_bag.py\nanalyze_mavros_bag.py\nLaTeX report", 16), "#ecfccb")

    add_arrow(ax, (0.22, 0.77), (0.28, 0.77), "MANUAL_CONTROL", text_xy=(0.245, 0.80))
    add_arrow(ax, (0.46, 0.77), (0.52, 0.77), "mode / joystick", text_xy=(0.49, 0.80))
    add_arrow(ax, (0.72, 0.75), (0.78, 0.75), "SERVO_OUTPUT_RAW", text_xy=(0.75, 0.77), rad=0.0)
    add_arrow(ax, (0.78, 0.69), (0.72, 0.69), "JSON 9003 / 9002", text_xy=(0.75, 0.64), rad=0.0)

    add_arrow(ax, (0.23, 0.32), (0.55, 0.32), "", text_xy=(0.39, 0.39))
    add_arrow(ax, (0.39, 0.54), (0.39, 0.30), "", text_xy=(0.43, 0.41))
    add_arrow(ax, (0.48, 0.42), (0.55, 0.42), "", text_xy=(0.515, 0.455))
    add_arrow(ax, (0.73, 0.40), (0.79, 0.40), "", text_xy=(0.76, 0.43))
    add_arrow(ax, (0.73, 0.55), (0.73, 0.42), "", text_xy=(0.75, 0.50))

    ax.text(
        0.5,
        0.97,
        "UUV MuJoCo v3.0 전체 시스템 블록다이어그램",
        ha="center",
        va="top",
        fontsize=16,
        fontweight="bold",
        color="#111827",
    )
    ax.text(
        0.5,
        0.935,
        "QGC, QGC proxy, ArduSub SITL, MuJoCo runtime, ROS 패키지, 측정 파이프라인을 한 화면에 정리한 그림",
        ha="center",
        va="top",
        fontsize=10,
        color="#475569",
    )
    fig.savefig(FIG_DIR / "uuv_v30_system_block_diagram.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_sitl_comm_diagram() -> None:
    fig, ax = plt.subplots(figsize=(14.0, 7.8), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    add_round_box(ax, 0.06, 0.63, 0.18, 0.20, "ArduSub JSON SITL", wrapped("serial0 -> 5760\nserial1 -> 14552\nserial2 -> 14551\nJSON servo out 9002", 18), "#fee2e2")
    add_round_box(ax, 0.39, 0.64, 0.20, 0.18, "sitl_transport.py", wrapped("recvfrom_into()\nJSON sensor encoder\nMAVLink command helper", 18), "#dbeafe")
    add_round_box(ax, 0.73, 0.63, 0.18, 0.20, "ros2_bridge.py", wrapped("MAVROS surface\nTF / DVL / depth\nset_mode / arm / RC", 18), "#dcfce7")

    add_round_box(ax, 0.07, 0.25, 0.17, 0.18, "QGC Proxy", wrapped("14552 <-> 14550\nMANUAL_CONTROL axis latch", 18), "#ede9fe")
    add_round_box(ax, 0.35, 0.23, 0.22, 0.22, "MuJoCo Physics Loop", wrapped("scene + profile\nthruster shaping\nellipsoid fluid\nxfrc_applied + mj_step", 19), "#fef3c7")
    add_round_box(ax, 0.69, 0.22, 0.22, 0.24, "ROS2 / rospkg", wrapped("/mavros/*\n/cmd_vel\n/dvl/*\n/depth\nuuv_control_gui.py", 18), "#fce7f3")

    add_arrow(ax, (0.24, 0.73), (0.39, 0.73), "JSON servo packet\nUDP 9002")
    add_arrow(ax, (0.39, 0.68), (0.24, 0.68), "JSON sensor payload\nUDP 9003")
    add_arrow(ax, (0.59, 0.72), (0.73, 0.72), "state publish / service bridge")
    add_arrow(ax, (0.73, 0.66), (0.59, 0.66), "RC override / mode / arm")
    add_arrow(ax, (0.15, 0.43), (0.15, 0.63), "serial1 UDP 14552 / 14550")
    add_arrow(ax, (0.49, 0.45), (0.49, 0.64), "body state -> SITL vertical truth")
    add_arrow(ax, (0.57, 0.34), (0.69, 0.34), "ROS2 topics / services")
    add_arrow(ax, (0.69, 0.29), (0.57, 0.29), "cmd_vel / RC / setpoint")

    ax.text(0.5, 0.96, "MuJoCo-SITL 통신 블록다이어그램", ha="center", va="top", fontsize=16, fontweight="bold")
    ax.text(
        0.5,
        0.92,
        "JSON servo/state 경로와 MAVLink service/telemetry 경로를 분리해 표시했다. 현재 측정 스크립트는 serial2 UDP 14551을 통해 arm/set_mode/RC override를 보낸다.",
        ha="center",
        va="top",
        fontsize=10,
        color="#475569",
    )
    fig.savefig(FIG_DIR / "uuv_v30_sitl_comm_diagram.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_physics_pipeline() -> None:
    fig, ax = plt.subplots(figsize=(14.0, 8.2), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    steps = [
        (0.05, 0.69, 0.16, 0.18, "1. ArduSub 출력", "SERVO_OUTPUT_RAW / JSON servo\n모드 로직은 ArduSub 내부"),
        (0.25, 0.69, 0.16, 0.18, "2. Thruster shaping", "deadzone, slew, tau_up/down,\nreverse asymmetry, gain scale"),
        (0.45, 0.69, 0.18, 0.18, "3. Rigid-body terms", "body_components 기반 CoM/inertia\nbuoyancy point와 CoB torque"),
        (0.67, 0.69, 0.18, 0.18, "4. Hydrodynamics", "legacy: coefficient 6-DOF\ncurrent: MuJoCo ellipsoid fluid + runtime 보정"),
        (0.22, 0.36, 0.18, 0.18, "5. MuJoCo step", "data.ctrl + data.xfrc_applied\nmj_step()로 적분 / contact / constraint"),
        (0.46, 0.36, 0.18, 0.18, "6. Sensor export", "imu, dvl, depth, odom\nworld/base frame 정합"),
        (0.70, 0.36, 0.18, 0.18, "7. SITL feedback", "altitude / quaternion / velocity\nArduSub JSON sensor payload"),
    ]
    fills = ["#fee2e2", "#dbeafe", "#e0f2fe", "#dcfce7", "#fef3c7", "#fce7f3", "#ecfccb"]
    for (x, y, w, h, title, body), fc in zip(steps, fills):
        add_round_box(ax, x, y, w, h, title, wrapped(body, 20), fc)

    add_arrow(ax, (0.21, 0.78), (0.25, 0.78), "PWM / normalized cmd")
    add_arrow(ax, (0.41, 0.78), (0.45, 0.78), "force / torque request")
    add_arrow(ax, (0.63, 0.78), (0.67, 0.78), "active model select")
    add_arrow(ax, (0.76, 0.69), (0.76, 0.54), "fluid/contact + extra wrench")
    add_arrow(ax, (0.58, 0.45), (0.46, 0.45), "pose / twist / sensordata", rad=0.0)
    add_arrow(ax, (0.64, 0.45), (0.70, 0.45), "JSON SITL state", rad=0.0)
    add_arrow(ax, (0.79, 0.36), (0.79, 0.24), "ArduSub update")

    ax.text(0.5, 0.96, "v3.0 물리엔진 계산 파이프라인", ha="center", va="top", fontsize=16, fontweight="bold")
    ax.text(
        0.5,
        0.92,
        "현재 모델은 순수 MuJoCo only가 아니라, Python runtime이 force/torque를 만들고 MuJoCo가 적분하는 하이브리드 구조다.",
        ha="center",
        va="top",
        fontsize=10,
        color="#475569",
    )
    fig.savefig(FIG_DIR / "uuv_v30_physics_pipeline.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_actual_mode_xy_compare(legacy: dict, current: dict) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 10.4), dpi=220, constrained_layout=True)
    for ax, mode_key in zip(axes.flat, MODE_ORDER):
        lx, ly, _ = mode_arrays(legacy, mode_key)
        cx, cy, _ = mode_arrays(current, mode_key)
        ax.plot(ly, lx, color=MODEL_COLORS["legacy"], linewidth=2.0, label="Legacy")
        ax.plot(cy, cx, color=MODEL_COLORS["current"], linewidth=2.0, label="Current v3.0")
        ax.scatter([ly[0], cy[0]], [lx[0], cx[0]], color=["#a855f7", "#0f766e"], s=18, zorder=3)
        ax.set_title(MODE_LABELS[mode_key])
        ax.set_xlabel("Y position [m]")
        ax.set_ylabel("X position [m]")
        ax.grid(True, alpha=0.25)
        ax.axis("equal")
    handles = [
        plt.Line2D([0], [0], color=MODEL_COLORS["legacy"], linewidth=2.2, label="Legacy"),
        plt.Line2D([0], [0], color=MODEL_COLORS["current"], linewidth=2.2, label="Current v3.0"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.01))
    fig.suptitle("실측 rosbag 기반 mode-path XY 비교", y=1.02, fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / "uuv_v30_actual_mode_xy_compare.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_actual_mode_3d_compare(legacy: dict, current: dict) -> None:
    fig = plt.figure(figsize=(13.4, 6.4), dpi=220, constrained_layout=True)
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    for ax, title, measurement in ((ax1, "Legacy", legacy), (ax2, "Current v3.0", current)):
        for mode_key in MODE_ORDER:
            x, y, depth = mode_arrays(measurement, mode_key)
            ax.plot(y, x, depth, color=MODE_COLORS[mode_key], linewidth=1.8, label=MODE_LABELS[mode_key])
        ax.set_title(title)
        ax.set_xlabel("Y [m]")
        ax.set_ylabel("X [m]")
        ax.set_zlabel("Depth [m]")
        ax.view_init(elev=24, azim=-56)
        ax.grid(True, alpha=0.20)
    handles = [plt.Line2D([0], [0], color=MODE_COLORS[m], linewidth=2.0, label=MODE_LABELS[m]) for m in MODE_ORDER]
    fig.legend(handles=handles, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.02))
    fig.suptitle("실측 rosbag 기반 3D trajectory 비교", y=1.04, fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / "uuv_v30_actual_mode_3d_compare.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_actual_metric_compare(legacy: dict, current: dict) -> None:
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
        "Trajectory path [m]": ("path", vals(legacy, "path"), vals(current, "path")),
        "Horizontal distance [m]": ("horiz", vals(legacy, "horiz"), vals(current, "horiz")),
        "Max |pitch| [deg]": ("pitch", vals(legacy, "pitch"), vals(current, "pitch")),
        "Max |roll| [deg]": ("roll", vals(legacy, "roll"), vals(current, "roll")),
    }

    fig, axes = plt.subplots(2, 2, figsize=(13.2, 9.2), dpi=220, constrained_layout=True)
    for ax, (title, (_, legacy_vals, current_vals)) in zip(axes.flat, metrics.items()):
        ax.bar(x - width / 2, legacy_vals, width=width, color=MODEL_COLORS["legacy"], label="Legacy")
        ax.bar(x + width / 2, current_vals, width=width, color=MODEL_COLORS["current"], label="Current v3.0")
        ax.set_title(title)
        ax.set_xticks(x, labels)
        ax.grid(True, axis="y", alpha=0.25)
    handles = [
        plt.Line2D([0], [0], color=MODEL_COLORS["legacy"], linewidth=8, label="Legacy"),
        plt.Line2D([0], [0], color=MODEL_COLORS["current"], linewidth=8, label="Current v3.0"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.01))
    fig.suptitle("실측 rosbag 지표 비교", y=1.02, fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / "uuv_v30_actual_metric_compare.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_profile_compare(profiles: dict) -> None:
    legacy = profiles["legacy"]
    current = profiles["current"]
    metrics = [
        ("Buoyancy scale", float(legacy["buoyancy_scale"]), float(current["buoyancy_scale"])),
        ("Surface heave damping", float(legacy["surface_heave_damping"]), float(current["surface_heave_damping"])),
        ("CoB torque scale", float(legacy["cob_torque_scale"]), float(current["cob_torque_scale"])),
        ("Thruster force max", float(legacy["thruster_force_max"]), float(current["thruster_force_max"])),
        ("Linear drag", float(legacy["linear_drag"]), float(current["linear_drag"])),
        ("Angular drag", float(legacy["angular_drag"]), float(current["angular_drag"])),
    ]
    inertia_legacy = float(np.mean(np.asarray(legacy.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]), dtype=float)))
    inertia_current = float(np.mean(np.asarray(current.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]), dtype=float)))
    metrics.append(("Body inertia scale", inertia_legacy, inertia_current))

    labels = [m[0] for m in metrics]
    legacy_vals = [m[1] for m in metrics]
    current_vals = [m[2] for m in metrics]
    x = np.arange(len(metrics))
    width = 0.36

    fig, ax = plt.subplots(figsize=(13.5, 6.6), dpi=220, constrained_layout=True)
    ax.bar(x - width / 2, legacy_vals, width=width, color=MODEL_COLORS["legacy"], label="Legacy")
    ax.bar(x + width / 2, current_vals, width=width, color=MODEL_COLORS["current"], label="Current v3.0")
    ax.set_xticks(x, labels, rotation=18, ha="right")
    ax.set_ylabel("Profile value")
    ax.set_title("legacy vs current profile knob 비교")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.savefig(FIG_DIR / "uuv_v30_profile_compare.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_ellipsoid_surface(ax, center: np.ndarray, radii: np.ndarray, color: str, alpha: float = 0.18) -> None:
    u = np.linspace(0.0, 2.0 * math.pi, 36)
    v = np.linspace(0.0, math.pi, 18)
    x = center[0] + radii[0] * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radii[1] * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radii[2] * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(y, x, z, color=color, alpha=alpha, linewidth=0.0, shade=False)


def plot_ellipsoid_layout(scene: dict, profiles: dict) -> None:
    current = profiles["current"]
    summary = composite_summary(current)

    fig = plt.figure(figsize=(11.8, 8.6), dpi=220, constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")

    for geom in scene["fluid_geoms"]:
        plot_ellipsoid_surface(ax, geom["pos"], geom["size"], "#60a5fa", alpha=0.12)
        pos = geom["pos"]
        ax.text(pos[1], pos[0], pos[2] + 0.03, geom["name"].replace("fluid_", ""), fontsize=8, color="#1d4ed8")

    for comp in current["body_components"]:
        pos = np.asarray(comp["mass_pos"], dtype=float)
        ax.scatter(pos[1], pos[0], pos[2], color="#ef4444", s=28)
        ax.text(pos[1], pos[0], pos[2] + 0.02, comp["name"], fontsize=8, color="#7f1d1d")

    for point in current["buoyancy_points"]:
        pos = np.asarray(point["pos"], dtype=float)
        ax.scatter(pos[1], pos[0], pos[2], color="#16a34a", s=24)

    com = summary["com"]
    cob = summary["buoy_center"]
    ax.scatter(com[1], com[0], com[2], color="black", s=50, marker="x")
    ax.scatter(cob[1], cob[0], cob[2], color="#059669", s=50, marker="^")
    ax.text(com[1], com[0], com[2] + 0.025, "CoM", fontsize=9, color="black")
    ax.text(cob[1], cob[0], cob[2] + 0.025, "CoB", fontsize=9, color="#065f46")

    for name, actuator in scene["actuators"].items():
        site_name = actuator["site"]
        if not site_name or site_name not in scene["sites"]:
            continue
        pos = scene["sites"][site_name]
        gear = actuator["gear"]
        norm = np.linalg.norm(gear)
        if norm <= 1e-9:
            continue
        vec = 0.08 * (gear / norm)
        ax.quiver(pos[1], pos[0], pos[2], vec[1], vec[0], vec[2], color="#0f172a", linewidth=1.1, arrow_length_ratio=0.18)

    ax.set_xlabel("Y [m]")
    ax.set_ylabel("X [m]")
    ax.set_zlabel("Z [m]")
    ax.view_init(elev=24, azim=-58)
    ax.set_title("current scene의 ellipsoid fluid / body component / CoM-CoB layout")
    fig.savefig(FIG_DIR / "uuv_v30_ellipsoid_layout.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def write_summary_json(legacy: dict, current: dict, profiles: dict, step_summary: dict) -> None:
    def average_metric(measurement: dict, key: str) -> float:
        vals = []
        for mode_key in MODE_ORDER:
            mode = measurement["modes"][mode_key]
            if key == "path":
                vals.append(float(mode["summary"]["trajectory_path_length_m"]))
            elif key == "horiz":
                vals.append(float(mode["summary"]["horizontal_distance_m"]))
            elif key == "pitch":
                vals.append(float(mode["summary"]["max_abs_pitch_deg"]))
            elif key == "roll":
                vals.append(float(mode["summary"]["max_abs_roll_deg"]))
            elif key == "yaw_delta":
                vals.append(abs(float(mode["segments"]["turn_90"]["yaw_delta_deg"])))
        return float(sum(vals) / len(vals))

    payload = {
        "actual_bag_paths": {
            "legacy_mode_path": str(LEGACY_MEAS_PATH),
            "current_mode_path": str(CURRENT_MEAS_PATH),
            "current_step_summary": str(CURRENT_STEP_SUMMARY_PATH),
        },
        "averages": {
            "legacy": {
                "path_m": average_metric(legacy, "path"),
                "horizontal_m": average_metric(legacy, "horiz"),
                "pitch_deg": average_metric(legacy, "pitch"),
                "roll_deg": average_metric(legacy, "roll"),
                "yaw_delta_deg": average_metric(legacy, "yaw_delta"),
            },
            "current": {
                "path_m": average_metric(current, "path"),
                "horizontal_m": average_metric(current, "horiz"),
                "pitch_deg": average_metric(current, "pitch"),
                "roll_deg": average_metric(current, "roll"),
                "yaw_delta_deg": average_metric(current, "yaw_delta"),
            },
        },
        "reductions_percent": {},
        "profiles": {
            "legacy": profiles["legacy"],
            "current": profiles["current"],
        },
        "current_step_summary": step_summary.get("summary", {}),
    }

    for key in ("path_m", "horizontal_m", "pitch_deg", "roll_deg", "yaw_delta_deg"):
        payload["reductions_percent"][key] = percent_reduction(payload["averages"]["legacy"][key], payload["averages"]["current"][key])

    SUMMARY_JSON_PATH.write_text(json.dumps(payload, indent=2, ensure_ascii=False))


def main() -> None:
    ensure_dirs()
    profiles = load_profiles()
    legacy_meas, current_meas = load_measurements()
    step_summary = load_step_summary()
    scene = parse_scene(CURRENT_SCENE)

    plot_system_block_diagram()
    plot_sitl_comm_diagram()
    plot_physics_pipeline()
    plot_actual_mode_xy_compare(legacy_meas, current_meas)
    plot_actual_mode_3d_compare(legacy_meas, current_meas)
    plot_actual_metric_compare(legacy_meas, current_meas)
    plot_profile_compare(profiles)
    plot_ellipsoid_layout(scene, profiles)
    write_summary_json(legacy_meas, current_meas, profiles, step_summary)


if __name__ == "__main__":
    main()

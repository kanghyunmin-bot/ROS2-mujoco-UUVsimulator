from __future__ import annotations

import json
import math
import re
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


ROOT = Path(__file__).resolve().parents[2]
DOCSRC_DIR = Path(__file__).resolve().parent
METRICS_DIR = DOCSRC_DIR / "metrics" / "report_inputs"
FIG_DIR = DOCSRC_DIR / "figures_v22"
SIM_DIR = ROOT / "uuv_mujoco" / "v2.2"
CONFIG_DIR = SIM_DIR / "config"
SCENE_PATH = SIM_DIR / "scenes" / "tank_current_scene.xml"
START_SCRIPT_PATH = SIM_DIR / "start_ardusub_sitl_mj311.sh"
THRUSTER_MAPPING_PATH = SIM_DIR / "physics" / "thruster_mapping.py"
MEAS_CUSTOM_PATH = METRICS_DIR / "measurement_custom_mode_path_latest.json"
MEAS_ELLIPSOID_PATH = METRICS_DIR / "measurement_ellipsoid_mode_path_latest.json"
ENGINE_METRICS_PATH = METRICS_DIR / "engine_comparison_metrics.json"

import sys

sys.path.insert(0, str(SIM_DIR))

from physics.hydrodynamics_helpers import scaled_polynomial_force, shape_thruster_command, submerged_fraction
from physics.sim_profile_helpers import build_hydrodynamics_config, build_sim_profile, load_sim_profiles


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Apple SD Gothic Neo", "Noto Sans CJK KR", "Arial", "DejaVu Sans"],
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "figure.titlesize": 13,
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


def ensure_dirs() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def load_profiles() -> tuple[dict, dict, dict]:
    profiles, warning = load_sim_profiles(CONFIG_DIR / "sim_profiles.json")
    if warning is not None:
        raise RuntimeError(warning)
    custom = build_sim_profile(profiles, "custom")
    ellipsoid = build_sim_profile(profiles, "ellipsoid")
    return profiles, custom, ellipsoid


def build_hydro_configs() -> tuple[dict, dict]:
    _, custom, ellipsoid = load_profiles()
    return (
        build_hydrodynamics_config(custom, fluid_density=1000.0),
        build_hydrodynamics_config(ellipsoid, fluid_density=1000.0),
    )


def parse_scene() -> dict:
    root = ET.parse(SCENE_PATH).getroot()
    base = root.find(".//body[@name='base_link']")
    if base is None:
        raise RuntimeError("base_link body not found in tank_current_scene.xml")

    inertial = base.find("inertial")
    actuators = {}
    for motor in root.findall(".//motor"):
        name = motor.attrib.get("name", "")
        gear = [float(v) for v in motor.attrib.get("gear", "0 0 0 0 0 0").split()[:3]]
        actuators[name] = {
            "site": motor.attrib.get("site", ""),
            "gear": np.array(gear, dtype=float),
        }

    sites: dict[str, np.ndarray] = {}
    for site in base.findall("site"):
        name = site.attrib.get("name", "")
        pos = np.array([float(v) for v in site.attrib.get("pos", "0 0 0").split()], dtype=float)
        sites[name] = pos

    fluid_geoms = []
    for geom in base.findall("geom"):
        name = geom.attrib.get("name", "")
        if not name.startswith("fluid_"):
            continue
        fluid_geoms.append(
            {
                "name": name,
                "type": geom.attrib.get("type", ""),
                "pos": np.array([float(v) for v in geom.attrib.get("pos", "0 0 0").split()], dtype=float),
                "size": np.array([float(v) for v in geom.attrib.get("size", "0 0 0").split()], dtype=float),
                "fluidcoef": np.array([float(v) for v in geom.attrib.get("fluidcoef", "0 0 0 0 0").split()], dtype=float),
            }
        )

    return {
        "inertial_mass": float(inertial.attrib.get("mass", "0")) if inertial is not None else 0.0,
        "inertial_pos": np.array([float(v) for v in inertial.attrib.get("pos", "0 0 0").split()], dtype=float)
        if inertial is not None
        else np.zeros(3),
        "diaginertia": np.array([float(v) for v in inertial.attrib.get("diaginertia", "0 0 0").split()], dtype=float)
        if inertial is not None
        else np.zeros(3),
        "sites": sites,
        "actuators": actuators,
        "fluid_geoms": fluid_geoms,
    }


def parse_sitl_params() -> dict[str, str]:
    text = START_SCRIPT_PATH.read_text()
    found = re.findall(r'append_param_if_not_overridden "([^"]+)" "([^"]+)"', text)
    return {name: value for name, value in found}


def composite_summary(profile: dict) -> dict:
    components = profile["body_components"]
    total_mass = float(sum(component["mass"] for component in components))
    com = np.zeros(3, dtype=float)
    for comp in components:
        com += float(comp["mass"]) * np.array(comp["mass_pos"], dtype=float)
    com /= total_mass

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
        offset = np.array(comp["mass_pos"], dtype=float) - com
        parallel_axis = m * np.array(
            [
                offset[1] ** 2 + offset[2] ** 2,
                offset[0] ** 2 + offset[2] ** 2,
                offset[0] ** 2 + offset[1] ** 2,
            ],
            dtype=float,
        )
        inertia += self_inertia + parallel_axis

    scale = np.array(profile.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]), dtype=float)
    buoyancy_points = profile.get("buoyancy_points", [])
    total_share = float(sum(point["share"] for point in buoyancy_points))
    buoy_center = np.zeros(3, dtype=float)
    if total_share > 0.0:
        for point in buoyancy_points:
            buoy_center += float(point["share"]) * np.array(point["pos"], dtype=float)
        buoy_center /= total_share
    buoy_center[0] += float(profile.get("cob_x_offset", 0.0))
    buoy_center[2] += float(profile.get("cob_z_offset", 0.0))

    return {
        "mass_total": total_mass,
        "com": com,
        "inertia_raw": inertia,
        "inertia_scaled": inertia * scale,
        "buoy_center": buoy_center,
    }


def draw_box(
    ax,
    xy,
    wh,
    title,
    body,
    facecolor,
    edgecolor="#334155",
    title_color="#0f172a",
    title_fontsize: float = 11.0,
    body_fontsize: float = 8.8,
) -> None:
    x, y = xy
    w, h = wh
    patch = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.012,rounding_size=0.03",
        linewidth=1.4,
        facecolor=facecolor,
        edgecolor=edgecolor,
    )
    ax.add_patch(patch)
    title_lines = max(1, len(str(title).splitlines()))
    body_lines = max(1, len(str(body).splitlines()))
    title_width_capacity = max(12.0, 18.0 * (w / 0.22))
    body_width_capacity = max(14.0, 22.0 * (w / 0.22))
    title_max_len = max(len(line) for line in str(title).splitlines())
    body_max_len = max(len(line) for line in str(body).splitlines())
    effective_title_fs = min(
        title_fontsize,
        max(
            8.0,
            title_fontsize
            - 0.18 * max(len(str(title)) - 18, 0) / 4.0
            - 0.09 * max(title_max_len - title_width_capacity, 0),
        ),
    )
    effective_body_fs = min(
        body_fontsize,
        max(
            6.6,
            body_fontsize
            - 0.32 * max(body_lines - 3, 0)
            - 0.08 * max(body_max_len - body_width_capacity, 0),
        ),
    )

    title_y = y + h * 0.75
    body_y = y + h * 0.43
    if title_lines >= 2 or body_lines >= 4:
        title_y = y + h * 0.78
        body_y = y + h * 0.40
    if body_lines >= 5:
        title_y = y + h * 0.80
        body_y = y + h * 0.39

    title_text = ax.text(
        x + w / 2.0,
        title_y,
        title,
        fontsize=effective_title_fs,
        fontweight="bold",
        color=title_color,
        ha="center",
        va="center",
        multialignment="center",
        wrap=True,
    )
    body_text = ax.text(
        x + w / 2.0,
        body_y,
        body,
        fontsize=effective_body_fs,
        color="#1f2937",
        ha="center",
        va="center",
        multialignment="center",
        wrap=True,
    )
    title_text.set_clip_path(patch)
    body_text.set_clip_path(patch)


def arrow(ax, p0, p1, color="#475569", text: str | None = None, text_offset=(0.0, 0.0)) -> None:
    ax.annotate(
        "",
        xy=p1,
        xytext=p0,
        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6, shrinkA=5, shrinkB=5),
    )
    if text:
        mx = 0.5 * (p0[0] + p1[0]) + text_offset[0]
        my = 0.5 * (p0[1] + p1[1]) + text_offset[1]
        wrapped = text if ("$" in text or "\\" in text) else textwrap.fill(text, width=20)
        ax.text(
            mx,
            my,
            wrapped,
            fontsize=8.0,
            color=color,
            ha="center",
            va="center",
            bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor="none", alpha=0.9),
        )


def set_axes_equal_3d(ax) -> None:
    x_limits = np.array(ax.get_xlim3d(), dtype=float)
    y_limits = np.array(ax.get_ylim3d(), dtype=float)
    z_limits = np.array(ax.get_zlim3d(), dtype=float)
    centers = np.array([x_limits.mean(), y_limits.mean(), z_limits.mean()], dtype=float)
    radius = 0.5 * max(x_limits[1] - x_limits[0], y_limits[1] - y_limits[0], z_limits[1] - z_limits[0])
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])
    ax.set_box_aspect((1.0, 1.0, 0.70))


def plot_ellipsoid_surface(ax, center: np.ndarray, radii: np.ndarray, color: str, alpha: float = 0.18) -> None:
    u = np.linspace(0.0, 2.0 * np.pi, 30)
    v = np.linspace(0.0, np.pi, 20)
    x = center[0] + radii[0] * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radii[1] * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radii[2] * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0.0, shade=False)
    ax.plot_wireframe(x, y, z, color=color, linewidth=0.35, alpha=min(alpha + 0.15, 0.45), rstride=4, cstride=4)


def draw_3d_rotation_ring(
    ax,
    center: np.ndarray,
    axis: np.ndarray,
    radius: float,
    color: str,
    label: str,
    label_scale: float = 1.28,
) -> None:
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    ref = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(np.dot(axis, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0], dtype=float)
    v1 = np.cross(axis, ref)
    v1 /= np.linalg.norm(v1)
    v2 = np.cross(axis, v1)
    theta = np.linspace(-0.35 * np.pi, 1.45 * np.pi, 140)
    ring = center[None, :] + radius * (
        np.cos(theta)[:, None] * v1[None, :] + np.sin(theta)[:, None] * v2[None, :]
    )
    ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color=color, linewidth=2.1)
    end = ring[-1]
    tangent = -np.sin(theta[-1]) * v1 + np.cos(theta[-1]) * v2
    tangent /= np.linalg.norm(tangent)
    ax.quiver(
        end[0],
        end[1],
        end[2],
        tangent[0],
        tangent[1],
        tangent[2],
        length=0.08,
        normalize=True,
        color=color,
        linewidth=2.2,
        arrow_length_ratio=0.35,
    )
    label_pos = center + label_scale * radius * (0.65 * v1 + 0.75 * v2)
    ax.text(label_pos[0], label_pos[1], label_pos[2], label, fontsize=9.2, color=color, fontweight="bold")


def plot_architecture_overview() -> None:
    fig, ax = plt.subplots(figsize=(12.0, 7.0), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    draw_box(
        ax,
        (0.04, 0.70),
        (0.22, 0.20),
        "1. User / GCS",
        "QGroundControl\nJoystick input\nMode switch",
        "#dbeafe",
    )
    draw_box(
        ax,
        (0.38, 0.70),
        (0.22, 0.20),
        "2. ArduSub SITL",
        "RC mapping\nMode logic\nATC / PSC control loops",
        "#fee2e2",
    )
    draw_box(
        ax,
        (0.72, 0.70),
        (0.22, 0.20),
        "3. MuJoCo Runtime",
        "run_urdf_full.py\nThruster shaping\nHydrodynamics\nROS2 publish",
        "#dcfce7",
    )

    draw_box(
        ax,
        (0.06, 0.36),
        (0.18, 0.16),
        "Config A",
        "start_ardusub_sitl_mj311.sh\nFCU parameters",
        "#fef3c7",
        body_fontsize=7.7,
    )
    draw_box(
        ax,
        (0.29, 0.36),
        (0.18, 0.16),
        "Config B",
        "thruster_params.json\nDeadzone / lag / gain",
        "#fef3c7",
        body_fontsize=7.7,
    )
    draw_box(
        ax,
        (0.52, 0.36),
        (0.18, 0.16),
        "Config C",
        "sim_profiles.json\nBuoyancy / drag / added mass",
        "#fef3c7",
        body_fontsize=7.6,
    )
    draw_box(
        ax,
        (0.75, 0.36),
        (0.18, 0.16),
        "Config D",
        "tank_current_scene.xml\nBody / sites / fluidcoef",
        "#fef3c7",
        body_fontsize=7.4,
    )

    draw_box(
        ax,
        (0.24, 0.05),
        (0.52, 0.19),
        "Outputs",
        "SERVO_OUTPUT_RAW -> motor force\nIMU / depth / DVL / odometry\nScene visualization and logs",
        "#ede9fe",
    )

    arrow(ax, (0.26, 0.80), (0.38, 0.80), "#2563eb", "RC / MAVLink")
    arrow(ax, (0.60, 0.80), (0.72, 0.80), "#dc2626", "servo + mode")
    arrow(ax, (0.83, 0.70), (0.83, 0.52), "#16a34a", "load at startup")
    arrow(ax, (0.61, 0.52), (0.79, 0.70), "#16a34a")
    arrow(ax, (0.38, 0.52), (0.49, 0.70), "#16a34a")
    arrow(ax, (0.15, 0.52), (0.49, 0.70), "#16a34a")
    arrow(ax, (0.84, 0.52), (0.84, 0.70), "#16a34a")
    arrow(ax, (0.49, 0.36), (0.49, 0.24), "#7c3aed", "runtime outputs")
    arrow(ax, (0.60, 0.36), (0.60, 0.24), "#7c3aed")

    ax.text(
        0.5,
        0.97,
        "UUV v2.2 end-to-end runtime architecture",
        ha="center",
        va="top",
        fontsize=14,
        fontweight="bold",
    )
    ax.text(
        0.5,
        0.935,
        "This figure answers one question: which file decides which behavior?",
        ha="center",
        va="top",
        fontsize=9.5,
        color="#475569",
    )

    fig.savefig(FIG_DIR / "uuv_v22_architecture.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_runtime_step_pipeline() -> None:
    fig, ax = plt.subplots(figsize=(13.2, 8.0), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    draw_box(
        ax,
        (0.04, 0.73),
        (0.22, 0.17),
        "1. Read MuJoCo state",
        "qpos / qvel\nbase pose, body vel\nworld depth, body rotation",
        "#dbeafe",
        body_fontsize=8.3,
    )
    draw_box(
        ax,
        (0.04, 0.46),
        (0.22, 0.18),
        "2. Thruster path",
        "servo / FCU input\n-> deadzone / limit\n-> first-order lag\n-> force model\n-> data.ctrl",
        "#fef3c7",
        body_fontsize=7.7,
    )
    draw_box(
        ax,
        (0.36, 0.73),
        (0.25, 0.17),
        "3. Hydrostatics",
        "distributed buoyancy\npoint-wise submerged fraction\nCoM/CoB torque\nsurface/full heave damping",
        "#dcfce7",
        body_fontsize=7.8,
    )
    draw_box(
        ax,
        (0.36, 0.46),
        (0.25, 0.18),
        "4. Custom-only hydrodynamics",
        "added mass\nadded-mass Coriolis\nlinear damping\nquadratic damping\n(body frame wrench)",
        "#fee2e2",
        body_fontsize=7.5,
    )
    draw_box(
        ax,
        (0.36, 0.17),
        (0.25, 0.18),
        "5. Runtime extra torques",
        "thruster reaction torque\nyaw torque scale on yaw thrusters\nbody->world rotation for wrench terms",
        "#fae8ff",
        body_fontsize=7.6,
    )
    draw_box(
        ax,
        (0.70, 0.59),
        (0.24, 0.20),
        "6. MuJoCo built-in physics",
        "actuator gear\njoint/body dynamics\ncontact and constraints\nfluid proxy drag\n(ellipsoid mode only)",
        "#e0f2fe",
        body_fontsize=7.5,
    )
    draw_box(
        ax,
        (0.70, 0.27),
        (0.24, 0.18),
        "7. Integrate one step",
        "data.xfrc_applied + data.ctrl\n-> mujoco.mj_step(model, data)\nnew pose, vel, sensor state",
        "#ede9fe",
        body_fontsize=7.9,
    )
    draw_box(
        ax,
        (0.70, 0.04),
        (0.24, 0.15),
        "8. Publish outward",
        "IMU / DVL / depth / odometry\nROS2 bridge and MAVROS compat topics",
        "#f1f5f9",
        body_fontsize=8.1,
    )

    arrow(ax, (0.26, 0.81), (0.36, 0.81), "#2563eb", "state -> hydrostatics", (0.0, 0.045))
    arrow(ax, (0.26, 0.55), (0.36, 0.55), "#d97706", "thrust path", (0.0, -0.045))
    arrow(ax, (0.26, 0.78), (0.36, 0.55), "#0f766e", "state -> damping", (-0.01, 0.035))
    arrow(ax, (0.26, 0.55), (0.36, 0.26), "#7c3aed", "side torques", (-0.01, -0.045))
    arrow(ax, (0.61, 0.81), (0.70, 0.69), "#16a34a", "hydro wrench", (0.015, 0.05))
    arrow(ax, (0.61, 0.55), (0.70, 0.63), "#dc2626", "custom wrench", (0.0, -0.045))
    arrow(ax, (0.61, 0.26), (0.70, 0.36), "#9333ea", "extra torques", (0.015, -0.045))
    arrow(ax, (0.26, 0.55), (0.70, 0.36), "#111827", "thrust -> data.ctrl", (0.0, -0.075))
    arrow(ax, (0.82, 0.59), (0.82, 0.45), "#2563eb", "mj_step", (0.055, 0.0))
    arrow(ax, (0.82, 0.27), (0.82, 0.19), "#475569", "sensor state", (0.07, 0.0))

    ax.text(
        0.5,
        0.97,
        "What the physics engine actually does every step",
        ha="center",
        va="top",
        fontsize=14,
        fontweight="bold",
    )
    ax.text(
        0.5,
        0.93,
        "Python runtime computes commands and extra wrenches; MuJoCo integrates the rigid-body dynamics and built-in fluid/contact terms.",
        ha="center",
        va="top",
        fontsize=9.3,
        color="#475569",
    )

    fig.savefig(FIG_DIR / "uuv_v22_runtime_step_pipeline.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_vehicle_layout_top() -> None:
    scene = parse_scene()
    _, _, ellipsoid = load_profiles()
    summary = composite_summary(ellipsoid)
    sites = scene["sites"]

    fig, ax = plt.subplots(figsize=(8.6, 7.2), dpi=220)
    ax.set_aspect("equal")

    for geom in scene["fluid_geoms"]:
        pos = geom["pos"]
        size = geom["size"]
        ell = patches.Ellipse(
            (pos[0], pos[1]),
            width=2 * size[0],
            height=2 * size[1],
            linewidth=1.5,
            edgecolor="#0284c7",
            facecolor="#bae6fd",
            alpha=0.35,
        )
        ax.add_patch(ell)
        ax.text(pos[0], pos[1], geom["name"].replace("fluid_", ""), fontsize=7.5, ha="center", va="center", color="#0f172a")

    for name in ("mass_center_enclosure", "mass_port_lower_body", "mass_starboard_lower_body"):
        pos = sites[name]
        ax.scatter(pos[0], pos[1], s=70, c="#dc2626", marker="o", edgecolors="white", linewidths=0.8, zorder=5)
        ax.text(pos[0] + 0.012, pos[1] + 0.012, name.replace("mass_", ""), fontsize=7.4, color="#7f1d1d")

    for name in ("buoy_ver_lf", "buoy_ver_lr", "buoy_ver_rf", "buoy_ver_rr"):
        pos = sites[name]
        ax.scatter(pos[0], pos[1], s=70, c="#16a34a", marker="s", edgecolors="white", linewidths=0.8, zorder=5)
        ax.text(pos[0] + 0.012, pos[1] - 0.018, name.replace("buoy_", ""), fontsize=7.2, color="#14532d")

    for name, actuator in scene["actuators"].items():
        if not name.startswith(("ver_", "yaw_")):
            continue
        site_name = actuator["site"]
        pos = sites.get(site_name, np.zeros(3))
        gear = actuator["gear"]
        if np.linalg.norm(gear[:2]) > 1e-9:
            direction = gear[:2] / np.linalg.norm(gear[:2])
            ax.arrow(
                pos[0],
                pos[1],
                0.055 * direction[0],
                0.055 * direction[1],
                width=0.003,
                head_width=0.018,
                head_length=0.022,
                length_includes_head=True,
                color="#111827",
                zorder=6,
            )
        else:
            ax.scatter(pos[0], pos[1], s=55, c="#111827", marker="^", zorder=6)
        ax.text(pos[0] - 0.012, pos[1] + 0.022, name, fontsize=7.1, color="#111827")

    com = summary["com"]
    buoy_center = summary["buoy_center"]
    ax.scatter(com[0], com[1], s=130, c="#111827", marker="X", zorder=7, label="Composite CoM")
    ax.scatter(buoy_center[0], buoy_center[1], s=130, c="#22c55e", marker="P", zorder=7, label="Effective buoyancy center")

    ax.axhline(0.0, color="#94a3b8", linewidth=0.8, linestyle=":")
    ax.axvline(0.0, color="#94a3b8", linewidth=0.8, linestyle=":")
    ax.set_xlabel("Body X [m] (forward +)")
    ax.set_ylabel("Body Y [m] (left +)")
    ax.set_title("Top view: fluid hulls, thrusters, mass points, buoyancy points")
    ax.grid(True, alpha=0.22)
    ax.legend(frameon=False, loc="upper right")

    ax.text(
        0.02,
        0.02,
        "Blue ellipses: built-in ellipsoid fluid proxy geoms\n"
        "Red dots: distributed mass components\n"
        "Green squares: buoyancy points above the vertical thrusters",
        transform=ax.transAxes,
        fontsize=8.2,
        va="bottom",
        bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="#cbd5e1", alpha=0.95),
    )

    fig.savefig(FIG_DIR / "uuv_v22_vehicle_layout_top.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_vehicle_layout_3d() -> None:
    scene = parse_scene()
    _, _, ellipsoid = load_profiles()
    summary = composite_summary(ellipsoid)
    sites = scene["sites"]

    fig = plt.figure(figsize=(11.2, 8.2), dpi=220)
    ax = fig.add_subplot(111, projection="3d")

    for geom in scene["fluid_geoms"]:
        plot_ellipsoid_surface(ax, geom["pos"], geom["size"], "#38bdf8", alpha=0.16)
        label_pos = geom["pos"] + np.array([0.0, 0.0, geom["size"][2] + 0.03])
        ax.text(
            label_pos[0],
            label_pos[1],
            label_pos[2],
            geom["name"].replace("fluid_", ""),
            fontsize=7.2,
            color="#0f172a",
            ha="center",
        )

    body_origin = np.zeros(3, dtype=float)
    axes_specs = [
        (np.array([0.18, 0.0, 0.0]), "#dc2626", "body +X"),
        (np.array([0.0, 0.18, 0.0]), "#2563eb", "body +Y"),
        (np.array([0.0, 0.0, 0.18]), "#16a34a", "body +Z"),
    ]
    for vec, color, label in axes_specs:
        ax.quiver(
            body_origin[0],
            body_origin[1],
            body_origin[2],
            vec[0],
            vec[1],
            vec[2],
            color=color,
            linewidth=2.6,
            arrow_length_ratio=0.14,
        )
        ax.text(*(vec + np.array([0.015, 0.015, 0.015])), label, color=color, fontsize=9.5, fontweight="bold")

    mass_names = ("mass_center_enclosure", "mass_port_lower_body", "mass_starboard_lower_body")
    buoy_names = ("buoy_ver_lf", "buoy_ver_lr", "buoy_ver_rf", "buoy_ver_rr")
    sensor_names = ("imu_site", "bar30_site", "dvl_site")

    for name in mass_names:
        pos = sites[name]
        ax.scatter(pos[0], pos[1], pos[2], s=58, c="#dc2626", marker="o", edgecolors="white", linewidths=0.5, depthshade=False)
        ax.text(pos[0] + 0.010, pos[1], pos[2] + 0.008, name.replace("mass_", ""), fontsize=7.0, color="#7f1d1d")

    for name in buoy_names:
        pos = sites[name]
        ax.scatter(pos[0], pos[1], pos[2], s=52, c="#16a34a", marker="s", edgecolors="white", linewidths=0.5, depthshade=False)
        ax.text(pos[0] + 0.010, pos[1], pos[2] + 0.008, name.replace("buoy_", ""), fontsize=6.8, color="#14532d")

    sensor_colors = {"imu_site": "#eab308", "bar30_site": "#06b6d4", "dvl_site": "#8b5cf6"}
    for name in sensor_names:
        pos = sites[name]
        ax.scatter(pos[0], pos[1], pos[2], s=60, c=sensor_colors[name], marker="D", edgecolors="white", linewidths=0.5, depthshade=False)
        ax.text(pos[0] + 0.010, pos[1], pos[2] - 0.010, name, fontsize=6.8, color="#0f172a")

    for name, actuator in scene["actuators"].items():
        if not name.startswith(("ver_", "yaw_")):
            continue
        site_name = actuator["site"]
        pos = sites.get(site_name, np.zeros(3))
        gear = np.array(actuator["gear"], dtype=float)
        if np.linalg.norm(gear) < 1e-9:
            continue
        direction = gear / np.linalg.norm(gear)
        color = "#111827" if name.startswith("ver_") else "#ea580c"
        length = 0.08 if name.startswith("ver_") else 0.10
        ax.quiver(
            pos[0],
            pos[1],
            pos[2],
            direction[0],
            direction[1],
            direction[2],
            length=length,
            normalize=True,
            color=color,
            linewidth=2.0,
            arrow_length_ratio=0.26,
        )
        if name.startswith("yaw_"):
            ax.text(pos[0] - 0.006, pos[1], pos[2] + 0.02, name, fontsize=6.7, color="#9a3412")

    com = summary["com"]
    buoy_center = summary["buoy_center"]
    ax.scatter(com[0], com[1], com[2], s=120, c="#111827", marker="X", depthshade=False)
    ax.text(com[0] + 0.015, com[1], com[2] + 0.012, "Composite CoM", fontsize=8.4, color="#111827", fontweight="bold")
    ax.scatter(buoy_center[0], buoy_center[1], buoy_center[2], s=115, c="#22c55e", marker="P", depthshade=False)
    ax.text(
        buoy_center[0] + 0.015,
        buoy_center[1],
        buoy_center[2] + 0.012,
        "Effective buoyancy center",
        fontsize=8.2,
        color="#166534",
        fontweight="bold",
    )

    ax.set_xlabel("Body X [m]")
    ax.set_ylabel("Body Y [m]")
    ax.set_zlabel("Body Z [m]")
    ax.view_init(elev=24, azim=-58)
    ax.grid(True, alpha=0.18)
    set_axes_equal_3d(ax)
    ax.text2D(
        0.03,
        0.97,
        "3D isometric layout: fluid proxy geoms, distributed mass, buoyancy points, sensors, and thruster directions",
        transform=ax.transAxes,
        fontsize=13.2,
        fontweight="bold",
        va="top",
    )
    ax.text2D(
        0.03,
        0.92,
        "This is the most direct picture for explaining trim, pitch bias, CoM/CoB offset, and why thrust planes matter.",
        transform=ax.transAxes,
        fontsize=9.0,
        color="#475569",
        va="top",
    )

    legend_handles = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#dc2626", markeredgecolor="white", markersize=8, label="mass component"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor="#16a34a", markeredgecolor="white", markersize=8, label="buoyancy point"),
        Line2D([0], [0], marker="D", color="w", markerfacecolor="#8b5cf6", markeredgecolor="white", markersize=7, label="sensor site"),
        Line2D([0], [0], marker="X", color="w", markerfacecolor="#111827", markeredgecolor="#111827", markersize=8, label="com / effective center"),
        Line2D([0], [0], color="#ea580c", lw=2.2, label="yaw thruster vector"),
        Line2D([0], [0], color="#111827", lw=2.2, label="vertical thruster vector"),
    ]
    ax.legend(handles=legend_handles, loc="lower left", bbox_to_anchor=(0.02, 0.02), frameon=True, framealpha=0.92)

    fig.savefig(FIG_DIR / "uuv_v22_vehicle_layout_3d.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_vehicle_layout_side() -> None:
    scene = parse_scene()
    _, _, ellipsoid = load_profiles()
    summary = composite_summary(ellipsoid)
    sites = scene["sites"]

    fig, ax = plt.subplots(figsize=(9.2, 6.4), dpi=220)

    for geom in scene["fluid_geoms"]:
        pos = geom["pos"]
        size = geom["size"]
        ell = patches.Ellipse(
            (pos[0], pos[2]),
            width=2 * size[0],
            height=2 * size[2],
            linewidth=1.5,
            edgecolor="#0284c7",
            facecolor="#bae6fd",
            alpha=0.35,
        )
        ax.add_patch(ell)

    for name in ("mass_center_enclosure", "mass_port_lower_body", "mass_starboard_lower_body"):
        pos = sites[name]
        ax.scatter(pos[0], pos[2], s=70, c="#dc2626", marker="o", edgecolors="white", linewidths=0.8)
        ax.text(pos[0] + 0.01, pos[2] + 0.005, name.replace("mass_", ""), fontsize=7.2, color="#7f1d1d")

    for name in ("buoy_ver_lf", "buoy_ver_lr", "buoy_ver_rf", "buoy_ver_rr"):
        pos = sites[name]
        ax.scatter(pos[0], pos[2], s=55, c="#16a34a", marker="s", edgecolors="white", linewidths=0.8)

    for name in ("thr_ver_lf", "thr_ver_lr", "thr_ver_rf", "thr_ver_rr"):
        pos = sites[name]
        ax.scatter(pos[0], pos[2], s=50, c="#111827", marker="v")

    for name, color in (("imu_site", "#eab308"), ("bar30_site", "#06b6d4"), ("dvl_site", "#8b5cf6")):
        pos = sites[name]
        ax.scatter(pos[0], pos[2], s=70, c=color, marker="D", edgecolors="white", linewidths=0.8)
        ax.text(pos[0] + 0.012, pos[2] - 0.005, name, fontsize=7.2, color="#0f172a")

    com = summary["com"]
    buoy_center = summary["buoy_center"]
    ax.scatter(com[0], com[2], s=130, c="#111827", marker="X", zorder=7, label="Composite CoM")
    ax.scatter(buoy_center[0], buoy_center[2], s=130, c="#22c55e", marker="P", zorder=7, label="Effective buoyancy center")
    ax.scatter(scene["inertial_pos"][0], scene["inertial_pos"][2], s=90, c="#f97316", marker="*", zorder=7, label="Scene inertial pos")

    ax.axhline(0.0, color="#0ea5e9", linewidth=1.2, linestyle="--", label="Body z = 0 plane")
    ax.axvline(0.0, color="#94a3b8", linewidth=0.8, linestyle=":")
    ax.set_xlabel("Body X [m] (forward +)")
    ax.set_ylabel("Body Z [m] (up +)")
    ax.set_title("Side view: mass distribution, buoyancy distribution, sensors, and inertial reference")
    ax.grid(True, alpha=0.22)
    ax.legend(frameon=False, loc="upper right", ncol=2)

    ax.text(
        0.02,
        0.02,
        "This view explains pitch behavior:\n"
        "if CoM, buoyancy center, thruster plane, and fluid drag center do not align,\n"
        "forward motion and depth hold create pitch moment.",
        transform=ax.transAxes,
        fontsize=8.2,
        va="bottom",
        bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="#cbd5e1", alpha=0.95),
    )

    fig.savefig(FIG_DIR / "uuv_v22_vehicle_layout_side.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def curved_arrow(ax, start, end, rad, color, label, text_pos):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops=dict(arrowstyle="-|>", lw=1.8, color=color, connectionstyle=f"arc3,rad={rad}"),
    )
    ax.text(text_pos[0], text_pos[1], label, fontsize=9.0, color=color, fontweight="bold", ha="center", va="center")


def plot_6dof_axes() -> None:
    fig, ax = plt.subplots(figsize=(10.6, 7.0), dpi=220)
    ax.set_xlim(-1.0, 1.2)
    ax.set_ylim(-0.8, 0.9)
    ax.set_aspect("equal")
    ax.axis("off")

    body = patches.FancyBboxPatch(
        (-0.35, -0.22),
        0.70,
        0.44,
        boxstyle="round,pad=0.02,rounding_size=0.05",
        linewidth=1.5,
        facecolor="#dbeafe",
        edgecolor="#1e3a8a",
    )
    ax.add_patch(body)
    ax.text(0.0, 0.0, "base_link", ha="center", va="center", fontsize=13, fontweight="bold", color="#0f172a")
    ax.scatter([0], [0], c="#111827", s=110, marker="X", zorder=6)
    ax.text(0.03, -0.06, "CoM", fontsize=9.0, color="#111827")

    ax.arrow(0.0, 0.0, 0.62, 0.0, width=0.008, head_width=0.05, head_length=0.07, color="#dc2626", length_includes_head=True)
    ax.text(0.74, 0.03, "surge +X", color="#dc2626", fontsize=10, fontweight="bold")

    ax.arrow(0.0, 0.0, 0.0, 0.50, width=0.008, head_width=0.05, head_length=0.07, color="#2563eb", length_includes_head=True)
    ax.text(0.03, 0.58, "sway +Y", color="#2563eb", fontsize=10, fontweight="bold")

    ax.arrow(0.0, 0.0, -0.48, -0.48, width=0.008, head_width=0.05, head_length=0.07, color="#16a34a", length_includes_head=True)
    ax.text(-0.82, -0.57, "heave +Z\n(up in body frame)", color="#16a34a", fontsize=9.5, fontweight="bold", ha="left")

    curved_arrow(ax, (-0.18, 0.28), (0.18, 0.28), -0.8, "#f97316", "roll p", (0.0, 0.54))
    curved_arrow(ax, (0.44, -0.05), (0.44, 0.30), 0.8, "#7c3aed", "pitch q", (0.73, 0.16))
    curved_arrow(ax, (-0.05, -0.42), (0.22, -0.35), -0.6, "#0f766e", "yaw r", (0.22, -0.62))

    draw_box(
        ax,
        (0.55, -0.62),
        (0.50, 0.44),
        "6-DOF state vector",
        r"$\nu = [u, v, w, p, q, r]^T$" "\n"
        "u,v,w : body-frame linear velocity\n"
        "p,q,r : body-frame angular velocity",
        "#fef3c7",
        body_fontsize=8.0,
    )

    ax.text(
        0.02,
        0.95,
        "Body axes and 6-DOF states used by the simulator",
        transform=ax.transAxes,
        fontsize=14,
        fontweight="bold",
        va="top",
    )
    ax.text(
        0.02,
        0.90,
        "All runtime hydrodynamic terms are built around this body-frame ordering: [u v w p q r].",
        transform=ax.transAxes,
        fontsize=9.2,
        color="#475569",
        va="top",
    )

    fig.savefig(FIG_DIR / "uuv_v22_6dof_axes.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_6dof_forces_3d() -> None:
    fig = plt.figure(figsize=(10.8, 8.0), dpi=220)
    ax = fig.add_subplot(111, projection="3d")

    plot_ellipsoid_surface(ax, np.array([0.0, 0.0, 0.0]), np.array([0.26, 0.18, 0.10]), "#93c5fd", alpha=0.12)
    plot_ellipsoid_surface(ax, np.array([-0.12, 0.18, -0.04]), np.array([0.15, 0.08, 0.07]), "#7dd3fc", alpha=0.10)
    plot_ellipsoid_surface(ax, np.array([-0.12, -0.18, -0.04]), np.array([0.15, 0.08, 0.07]), "#7dd3fc", alpha=0.10)

    center = np.zeros(3, dtype=float)
    arrows = [
        (np.array([0.28, 0.0, 0.0]), "#dc2626", r"$F_x$, surge u"),
        (np.array([0.0, 0.28, 0.0]), "#2563eb", r"$F_y$, sway v"),
        (np.array([0.0, 0.0, 0.28]), "#16a34a", r"$F_z$, heave w"),
    ]
    for vec, color, label in arrows:
        ax.quiver(center[0], center[1], center[2], vec[0], vec[1], vec[2], color=color, linewidth=2.8, arrow_length_ratio=0.12)
        ax.text(*(vec + np.array([0.02, 0.02, 0.02])), label, fontsize=10.0, color=color, fontweight="bold")

    draw_3d_rotation_ring(ax, center + np.array([0.0, 0.0, 0.14]), np.array([1.0, 0.0, 0.0]), 0.16, "#f97316", r"$\tau_x$, roll p")
    draw_3d_rotation_ring(ax, center + np.array([0.18, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), 0.16, "#7c3aed", r"$\tau_y$, pitch q")
    draw_3d_rotation_ring(ax, center + np.array([0.0, 0.0, -0.16]), np.array([0.0, 0.0, 1.0]), 0.18, "#0f766e", r"$\tau_z$, yaw r")

    ax.scatter(0.0, 0.0, 0.0, s=120, c="#111827", marker="X", depthshade=False)
    ax.text(0.015, -0.02, 0.015, "CoM / wrench reference", fontsize=8.8, color="#111827")

    ax.set_xlabel("Body X [m]")
    ax.set_ylabel("Body Y [m]")
    ax.set_zlabel("Body Z [m]")
    ax.view_init(elev=22, azim=-56)
    ax.grid(True, alpha=0.18)
    set_axes_equal_3d(ax)
    ax.text2D(
        0.03,
        0.97,
        "3D 6-DOF wrench view: the simulator finally builds these six quantities",
        transform=ax.transAxes,
        fontsize=13.2,
        fontweight="bold",
        va="top",
    )
    ax.text2D(
        0.03,
        0.92,
        r"$\tau = [F_x, F_y, F_z, \tau_x, \tau_y, \tau_z]^T$"
        "  and each runtime term contributes to one or more of these six channels.",
        transform=ax.transAxes,
        fontsize=9.1,
        color="#475569",
        va="top",
    )

    fig.savefig(FIG_DIR / "uuv_v22_6dof_forces_3d.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_buoyancy_model() -> None:
    _, _, ellipsoid = load_profiles()
    rho = 1000.0
    g = 9.81
    mass_total = composite_summary(ellipsoid)["mass_total"]
    half_height = float(ellipsoid.get("half_height", 0.147))
    neutral_volume = mass_total / rho
    scale = float(ellipsoid["buoyancy_scale"])

    depth = np.linspace(-half_height, half_height, 500)
    u = depth / half_height
    linear = np.clip((depth + half_height) / (2.0 * half_height), 0.0, 1.0)
    ell = np.array([submerged_fraction(float(d), half_height, "ellipsoid") for d in depth], dtype=float)
    linear_force = rho * g * neutral_volume * linear * scale
    ell_force = rho * g * neutral_volume * ell * scale
    linear_slope = np.gradient(linear_force, depth)
    ell_slope = np.gradient(ell_force, depth)

    fig, axes = plt.subplots(1, 3, figsize=(14.4, 4.4), dpi=220, constrained_layout=True)

    axes[0].plot(u, linear, color="#64748b", linestyle="--", linewidth=2.0, label="linear")
    axes[0].plot(u, ell, color="#2563eb", linewidth=2.2, label="ellipsoid")
    axes[0].set_title("Submerged ratio")
    axes[0].set_xlabel("normalized depth u = z / H")
    axes[0].set_ylabel("submerged fraction")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(frameon=False)

    axes[1].plot(depth, linear_force, color="#64748b", linestyle="--", linewidth=2.0, label="linear")
    axes[1].plot(depth, ell_force, color="#f97316", linewidth=2.2, label="ellipsoid")
    axes[1].set_title("Buoyancy force")
    axes[1].set_xlabel("depth from water surface [m]")
    axes[1].set_ylabel("force [N]")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(depth, linear_slope, color="#64748b", linestyle="--", linewidth=2.0, label="linear")
    axes[2].plot(depth, ell_slope, color="#16a34a", linewidth=2.2, label="ellipsoid")
    axes[2].set_title("Buoyancy slope dF/dz")
    axes[2].set_xlabel("depth from water surface [m]")
    axes[2].set_ylabel("slope [N/m]")
    axes[2].grid(True, alpha=0.25)

    fig.suptitle("Hydrostatic model used by v2.2: linear vs ellipsoid submerged-fraction model", y=1.03)
    fig.savefig(FIG_DIR / "uuv_v22_buoyancy_model.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def nearest_curve(curves: list[dict], voltage: float) -> dict:
    return min(curves, key=lambda curve: abs(float(curve["voltage_v"]) - voltage))


def first_order_step_response(target: float, tau: float, dt: float = 0.002, t_end: float = 0.30) -> tuple[np.ndarray, np.ndarray]:
    t = np.arange(0.0, t_end + dt, dt)
    y = np.zeros_like(t)
    for i in range(1, len(t)):
        y[i] = y[i - 1] + (target - y[i - 1]) * (dt / max(tau, 1e-6))
    return t, y


def plot_thruster_model() -> None:
    params = load_json(CONFIG_DIR / "thruster_params.json")
    perf = load_json(CONFIG_DIR / "thruster_performance.json")
    _, _, ellipsoid = load_profiles()

    global_cfg = params["global"]
    voltage = float(ellipsoid["thruster_voltage"])
    curve = nearest_curve(perf["curves"], voltage)
    pwm = np.asarray(curve["pwm_us"], dtype=float)
    force = np.asarray(curve["force_n"], dtype=float)
    neutral_pwm = float(perf["meta"]["neutral_pwm"])

    cmd = np.linspace(-1.0, 1.0, 500)
    shaped = np.array(
        [
            shape_thruster_command(float(c), float(global_cfg["deadzone"]), float(global_cfg["command_limit"]))
            for c in cmd
        ],
        dtype=float,
    )
    force_poly = np.zeros_like(cmd)
    forward_poly = global_cfg["forward_poly"]
    reverse_poly = global_cfg["reverse_poly"]
    force_max = float(ellipsoid["thruster_force_max"])
    gain_scale = float(global_cfg["gain_scale_all"])
    reverse_asymmetry = float(global_cfg["reverse_asymmetry"])
    for i, c in enumerate(shaped):
        if c >= 0.0:
            force_poly[i] = scaled_polynomial_force(float(abs(c)), forward_poly, force_max) * gain_scale
        else:
            force_poly[i] = -scaled_polynomial_force(float(abs(c)), reverse_poly, force_max * reverse_asymmetry) * gain_scale

    t_up, y_up = first_order_step_response(1.0, float(global_cfg["tau_up"]))
    t_down, y_down = first_order_step_response(0.0, float(global_cfg["tau_down"]))
    y_down = 1.0 - y_down

    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.6), dpi=220)

    axes[0, 0].plot(pwm, force, linewidth=2.2, color="#2563eb", label=f"nearest measured curve ({curve['voltage_v']} V)")
    axes[0, 0].axvline(neutral_pwm, color="#94a3b8", linewidth=1.0, linestyle=":")
    axes[0, 0].axhline(0.0, color="#cbd5e1", linewidth=0.8)
    axes[0, 0].set_title("Measured T200 curve")
    axes[0, 0].set_xlabel("PWM [us]")
    axes[0, 0].set_ylabel("Force [N]")
    axes[0, 0].grid(True, alpha=0.25)
    axes[0, 0].legend(frameon=False)

    axes[0, 1].plot(cmd, shaped, linewidth=2.2, color="#16a34a")
    axes[0, 1].axvline(float(global_cfg["deadzone"]), color="#cbd5e1", linestyle=":", linewidth=1.0)
    axes[0, 1].axvline(-float(global_cfg["deadzone"]), color="#cbd5e1", linestyle=":", linewidth=1.0)
    axes[0, 1].set_title("Command shaping after deadzone/limit")
    axes[0, 1].set_xlabel("normalized input")
    axes[0, 1].set_ylabel("shaped command")
    axes[0, 1].grid(True, alpha=0.25)

    axes[1, 0].plot(cmd, force_poly, linewidth=2.2, color="#f97316")
    axes[1, 0].axhline(0.0, color="#cbd5e1", linewidth=0.8)
    axes[1, 0].set_title("Fallback force model used in simple SITL path")
    axes[1, 0].set_xlabel("normalized input")
    axes[1, 0].set_ylabel("force [N]")
    axes[1, 0].grid(True, alpha=0.25)

    axes[1, 1].plot(t_up, y_up, linewidth=2.2, color="#7c3aed", label=f"rise tau={global_cfg['tau_up']}")
    axes[1, 1].plot(t_down, y_down, linewidth=2.2, color="#dc2626", label=f"fall tau={global_cfg['tau_down']}")
    axes[1, 1].set_title("First-order thruster lag")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("normalized response")
    axes[1, 1].grid(True, alpha=0.25)
    axes[1, 1].legend(frameon=False)

    fig.suptitle("Current thruster model: measured curve, shaping, polynomial force, and lag", y=0.97)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "uuv_v22_thruster_model.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_hydrodynamics_comparison() -> None:
    custom_cfg, ellipsoid_cfg = build_hydro_configs()
    labels_t = ["surge", "sway", "heave"]
    labels_r = ["roll", "pitch", "yaw"]
    width = 0.35
    x = np.arange(3)

    fig, axes = plt.subplots(2, 2, figsize=(12.8, 8.6), dpi=220)

    axes[0, 0].bar(x - width / 2, custom_cfg.added_mass_diag[:3], width, color="#fb7185", label="custom")
    axes[0, 0].bar(x + width / 2, ellipsoid_cfg.added_mass_diag[:3], width, color="#60a5fa", label="ellipsoid")
    axes[0, 0].set_xticks(x, labels_t)
    axes[0, 0].set_title("Translational added mass")
    axes[0, 0].grid(True, axis="y", alpha=0.25)

    axes[0, 1].bar(x - width / 2, custom_cfg.linear_damping_diag[:3], width, color="#fb7185", label="custom")
    axes[0, 1].bar(x + width / 2, ellipsoid_cfg.linear_damping_diag[:3], width, color="#60a5fa", label="ellipsoid")
    axes[0, 1].set_xticks(x, labels_t)
    axes[0, 1].set_title("Translational linear damping")
    axes[0, 1].grid(True, axis="y", alpha=0.25)

    axes[1, 0].bar(x - width / 2, custom_cfg.added_mass_diag[3:], width, color="#fb7185", label="custom")
    axes[1, 0].bar(x + width / 2, ellipsoid_cfg.added_mass_diag[3:], width, color="#60a5fa", label="ellipsoid")
    axes[1, 0].set_xticks(x, labels_r)
    axes[1, 0].set_title("Rotational added inertia / added mass")
    axes[1, 0].grid(True, axis="y", alpha=0.25)

    axes[1, 1].bar(x - width / 2, custom_cfg.linear_damping_diag[3:], width, color="#fb7185", label="custom")
    axes[1, 1].bar(x + width / 2, ellipsoid_cfg.linear_damping_diag[3:], width, color="#60a5fa", label="ellipsoid")
    axes[1, 1].set_xticks(x, labels_r)
    axes[1, 1].set_title("Rotational linear damping")
    axes[1, 1].grid(True, axis="y", alpha=0.25)

    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 0.98))
    fig.suptitle("Profile-level hydrodynamic summary derived at runtime", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(FIG_DIR / "uuv_v22_hydrodynamics_comparison.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_runtime_term_matrix() -> None:
    rows = [
        "Distributed mass/inertia synthesis",
        "Distributed buoyancy force",
        "Buoyancy torque from point offset",
        "Surface heave damping",
        "Full-submerged heave damping",
        "Added-mass inertia term",
        "Added-mass Coriolis term",
        "Linear damping",
        "Quadratic damping",
        "Built-in ellipsoid fluid proxy force",
        "Thruster reaction torque",
        "Yaw torque scale on yaw thrusters",
    ]
    cols = ["surge X", "sway Y", "heave Z", "roll p", "pitch q", "yaw r"]
    data = np.array(
        [
            [0, 0, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [0, 0, 0, 2, 2, 0],
            [0, 0, 2, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [2, 2, 2, 2, 2, 2],
            [2, 2, 2, 2, 2, 2],
            [2, 2, 2, 2, 2, 2],
            [2, 2, 2, 2, 2, 2],
            [2, 2, 2, 2, 2, 2],
            [0, 0, 0, 1, 1, 2],
            [0, 0, 0, 0, 0, 2],
        ],
        dtype=float,
    )

    fig, axes = plt.subplots(1, 2, figsize=(14.2, 8.2), dpi=220)
    im = axes[0].imshow(data, cmap="YlGnBu", vmin=0.0, vmax=2.0, aspect="auto")
    axes[0].set_xticks(np.arange(len(cols)), cols, rotation=35, ha="right")
    axes[0].set_yticks(np.arange(len(rows)), rows)
    axes[0].set_title("Which DOF each physical term mainly affects")
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            txt = {0.0: "", 1.0: "weak", 2.0: "main"}[data[i, j]]
            if txt:
                axes[0].text(j, i, txt, ha="center", va="center", fontsize=7.1, color="#0f172a")

    mode_rows = rows
    mode_cols = ["custom", "ellipsoid"]
    mode_data = np.array(
        [
            [1, 1],
            [1, 1],
            [1, 1],
            [1, 1],
            [0, 1],
            [1, 0],
            [1, 0],
            [1, 0],
            [1, 0],
            [0, 1],
            [1, 1],
            [0, 1],
        ],
        dtype=float,
    )
    axes[1].imshow(mode_data, cmap="Oranges", vmin=0.0, vmax=1.0, aspect="auto")
    axes[1].set_xticks(np.arange(len(mode_cols)), mode_cols)
    axes[1].set_yticks(np.arange(len(mode_rows)), mode_rows)
    axes[1].set_title("Which runtime path activates the term")
    for i in range(mode_data.shape[0]):
        for j in range(mode_data.shape[1]):
            axes[1].text(
                j,
                i,
                "ON" if mode_data[i, j] > 0.5 else "-",
                ha="center",
                va="center",
                fontsize=8.1,
                color="#0f172a",
                fontweight="bold" if mode_data[i, j] > 0.5 else None,
            )

    fig.colorbar(im, ax=axes.ravel().tolist(), shrink=0.78, pad=0.015, label="relative contribution level")
    fig.suptitle("Runtime force/torque term map: what acts on which DOF, and in which mode", y=0.99)
    fig.subplots_adjust(top=0.90, bottom=0.08, left=0.15, right=0.96, wspace=0.45)
    fig.savefig(FIG_DIR / "uuv_v22_runtime_term_matrix.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_custom_wrench_pipeline() -> None:
    fig, ax = plt.subplots(figsize=(12.6, 7.6), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    draw_box(ax, (0.03, 0.68), (0.25, 0.22), "Inputs", "body velocity\nwater current\nsubmerged fraction\nprofile coefficients", "#dbeafe", body_fontsize=8.1)
    draw_box(ax, (0.37, 0.72), (0.24, 0.14), "Hydrostatics", r"$F_B = \rho g V_{sub}$" "\npoint-wise buoyancy + torque", "#dcfce7", body_fontsize=8.1)
    draw_box(ax, (0.37, 0.48), (0.24, 0.16), "Added mass", r"$-M_A \dot{\nu}_r$" "\n" r"$-C_A(\nu_r)\nu_r$", "#fee2e2", body_fontsize=8.0)
    draw_box(ax, (0.37, 0.23), (0.24, 0.16), "Damping", r"$-D_1 \nu_r$" "\n" r"$-D_2 |\nu_r|\odot \nu_r$", "#fef3c7", body_fontsize=8.0)
    draw_box(ax, (0.70, 0.48), (0.25, 0.26), "Runtime wrench sum", "body-frame hydro wrench\n-> rotate to world frame\n-> add to xfrc_applied", "#ede9fe", body_fontsize=8.0)
    draw_box(ax, (0.70, 0.16), (0.25, 0.16), "Extra terms", "surface heave damping\nthruster reaction torque", "#fae8ff", body_fontsize=8.0)

    arrow(ax, (0.28, 0.79), (0.37, 0.79), "#2563eb", r"$V_{sub}$")
    arrow(ax, (0.28, 0.75), (0.37, 0.57), "#dc2626", r"$\nu_r,\ \dot{\nu}_r$")
    arrow(ax, (0.28, 0.72), (0.37, 0.31), "#d97706", r"$\nu_r$")
    arrow(ax, (0.61, 0.79), (0.70, 0.61), "#16a34a", r"$F_B,\ \tau_B$")
    arrow(ax, (0.61, 0.56), (0.70, 0.56), "#dc2626", r"$-M_A\dot{\nu}_r - C_A\nu_r$")
    arrow(ax, (0.61, 0.31), (0.70, 0.53), "#d97706", r"$-D_1\nu_r - D_2|\nu_r|\nu_r$")
    arrow(ax, (0.82, 0.48), (0.82, 0.32), "#7c3aed", "plus", (0.05, 0.0))

    ax.text(
        0.5,
        0.96,
        "Custom 6-DOF wrench model implemented in run_urdf_full.py",
        ha="center",
        va="top",
        fontsize=14,
        fontweight="bold",
    )
    ax.text(
        0.5,
        0.92,
        "This is the exact conceptual stack used before writing to data.xfrc_applied on the base body.",
        ha="center",
        va="top",
        fontsize=9.2,
        color="#475569",
    )

    fig.savefig(FIG_DIR / "uuv_v22_custom_wrench_pipeline.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_yaw_pipeline() -> None:
    params = parse_sitl_params()
    fig, ax = plt.subplots(figsize=(12.6, 6.6), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    draw_box(ax, (0.04, 0.60), (0.23, 0.22), "MANUAL yaw path", "RC yaw stick\n-> direct motor yaw request\nNo heading hold brake", "#dbeafe", body_fontsize=8.1)
    draw_box(ax, (0.39, 0.60), (0.23, 0.22), "ALT_HOLD / POSHOLD yaw path", "RC yaw stick\n-> desired yaw rate\n-> attitude controller\n-> heading hold when stick returns to center", "#fee2e2", body_fontsize=7.5)
    draw_box(ax, (0.74, 0.60), (0.22, 0.22), "MuJoCo realization", "Yaw thruster sites\nYaw torque scale\nScene drag + runtime damping", "#dcfce7", body_fontsize=7.9)

    draw_box(
        ax,
        (0.10, 0.20),
        (0.80, 0.22),
        "Current assisted-mode yaw tuning snapshot",
        f"RC4_DZ={params.get('RC4_DZ','?')}   "
        f"ACRO_YAW_P={params.get('ACRO_YAW_P','?')}   "
        f"ATC_ANG_YAW_P={params.get('ATC_ANG_YAW_P','?')}\n"
        f"ATC_ACC_Y_MAX={params.get('ATC_ACC_Y_MAX','?')}   "
        f"ATC_RATE_Y_MAX={params.get('ATC_RATE_Y_MAX','?')}\n"
        f"ATC_RAT_YAW_P/I/IMAX/D={params.get('ATC_RAT_YAW_P','?')}/"
        f"{params.get('ATC_RAT_YAW_I','?')}/{params.get('ATC_RAT_YAW_IMAX','?')}/"
        f"{params.get('ATC_RAT_YAW_D','?')}",
        "#fef3c7",
        title_fontsize=10.5,
        body_fontsize=7.8,
    )

    arrow(ax, (0.27, 0.71), (0.39, 0.71), "#2563eb", "assisted controller layers", (0.0, 0.06))
    arrow(ax, (0.62, 0.71), (0.74, 0.71), "#dc2626", "same stick, different path", (0.0, 0.055))
    arrow(ax, (0.50, 0.60), (0.50, 0.42), "#7c3aed", "controller gains", (0.075, 0.0))
    arrow(ax, (0.84, 0.60), (0.84, 0.42), "#16a34a", "physics authority", (0.08, 0.0))

    ax.text(
        0.5,
        0.95,
        "Why yaw can feel different between MANUAL and DEPTH_HOLD-like modes",
        ha="center",
        va="top",
        fontsize=14,
        fontweight="bold",
    )
    ax.text(
        0.5,
        0.90,
        "The key point: in assisted modes, yaw is no longer a direct thruster request. It becomes a rate target that passes through attitude control.",
        ha="center",
        va="top",
        fontsize=9.2,
        color="#475569",
    )

    fig.savefig(FIG_DIR / "uuv_v22_yaw_pipeline.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def load_measurements() -> tuple[dict, dict]:
    return load_json(MEAS_CUSTOM_PATH), load_json(MEAS_ELLIPSOID_PATH)


def mode_trajectory_arrays(mode_payload: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pts = mode_payload["summary"]["trajectory_xyz_depth"]
    x = np.asarray([point["x"] for point in pts], dtype=float)
    y = np.asarray([point["y"] for point in pts], dtype=float)
    d = np.asarray([point["depth"] for point in pts], dtype=float)
    return x, y, d


def plot_mode_paths_xy() -> None:
    custom, ellipsoid = load_measurements()
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 9.0), dpi=220)
    for ax, mode in zip(axes.ravel(), MODE_ORDER):
        c = custom["modes"][mode]
        e = ellipsoid["modes"][mode]
        cx, cy, _ = mode_trajectory_arrays(c)
        ex, ey, _ = mode_trajectory_arrays(e)
        ax.plot(cy, cx, color="#dc2626", linewidth=2.1, label="custom")
        ax.plot(ey, ex, color="#2563eb", linewidth=2.1, label="ellipsoid")
        ax.scatter([cy[0]], [cx[0]], color="#111827", s=18)
        ax.scatter([cy[-1]], [cx[-1]], color="#dc2626", s=26)
        ax.scatter([ey[-1]], [ex[-1]], color="#2563eb", s=26)
        ax.set_title(MODE_LABELS[mode])
        ax.set_xlabel("Y [m]")
        ax.set_ylabel("X [m]")
        ax.grid(True, alpha=0.25)
        ax.axis("equal")

    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", frameon=False, ncol=2, bbox_to_anchor=(0.5, 0.02))
    fig.suptitle("Measured XY path comparison by mode: custom vs ellipsoid", y=0.97)
    fig.subplots_adjust(top=0.90, bottom=0.10, left=0.08, right=0.98, wspace=0.24, hspace=0.28)
    fig.savefig(FIG_DIR / "uuv_v22_mode_paths_xy.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_mode_metrics() -> None:
    custom, ellipsoid = load_measurements()
    labels = [MODE_LABELS[m] for m in MODE_ORDER]
    x = np.arange(len(labels))
    width = 0.18

    c_path = [custom["modes"][m]["summary"]["trajectory_path_length_m"] for m in MODE_ORDER]
    e_path = [ellipsoid["modes"][m]["summary"]["trajectory_path_length_m"] for m in MODE_ORDER]
    c_pitch = [custom["modes"][m]["summary"]["max_abs_pitch_deg"] for m in MODE_ORDER]
    e_pitch = [ellipsoid["modes"][m]["summary"]["max_abs_pitch_deg"] for m in MODE_ORDER]
    c_yaw_err = [abs(abs(custom["modes"][m]["segments"]["turn_90"]["yaw_delta_deg"]) - 90.0) for m in MODE_ORDER]
    e_yaw_err = [abs(abs(ellipsoid["modes"][m]["segments"]["turn_90"]["yaw_delta_deg"]) - 90.0) for m in MODE_ORDER]

    fig, ax = plt.subplots(figsize=(12.6, 5.8), dpi=220)
    ax.bar(x - 2.5 * width, c_path, width=width, color="#fca5a5", label="custom path length [m]")
    ax.bar(x - 1.5 * width, e_path, width=width, color="#93c5fd", label="ellipsoid path length [m]")
    ax.bar(x - 0.5 * width, c_pitch, width=width, color="#ef4444", label="custom max |pitch| [deg]")
    ax.bar(x + 0.5 * width, e_pitch, width=width, color="#2563eb", label="ellipsoid max |pitch| [deg]")
    ax.bar(x + 1.5 * width, c_yaw_err, width=width, color="#fb7185", label="custom turn error [deg]")
    ax.bar(x + 2.5 * width, e_yaw_err, width=width, color="#0ea5e9", label="ellipsoid turn error [deg]")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_title("Measured mode metrics: path length, pitch disturbance, and 90-degree turn error")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(frameon=False, ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.16))
    fig.subplots_adjust(top=0.76, bottom=0.12, left=0.07, right=0.98)
    fig.savefig(FIG_DIR / "uuv_v22_mode_metrics.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_engine_snapshot() -> None:
    metrics = load_json(ENGINE_METRICS_PATH)
    order = ("legacy_custom", "current_custom", "current_ellipsoid")
    labels = ["Legacy\nCustom", "Current\nCustom", "Current\nEllipsoid"]
    x = np.arange(len(order))
    fig, axes = plt.subplots(1, 3, figsize=(13.2, 4.8), dpi=220, constrained_layout=True)

    forward_force = [metrics[key]["forward_force_n"] for key in order]
    peak_pitch = [metrics[key]["peak_pitch_deg"] for key in order]
    peak_yaw_rate = [metrics[key]["peak_yaw_rate_radps"] for key in order]

    axes[0].bar(x, forward_force, color=["#94a3b8", "#ef4444", "#2563eb"])
    axes[0].set_xticks(x, labels)
    axes[0].set_title("Forward force")
    axes[0].set_ylabel("N")
    axes[0].grid(True, axis="y", alpha=0.25)

    axes[1].bar(x, peak_pitch, color=["#94a3b8", "#ef4444", "#2563eb"])
    axes[1].set_xticks(x, labels)
    axes[1].set_title("Peak pitch disturbance")
    axes[1].set_ylabel("deg")
    axes[1].grid(True, axis="y", alpha=0.25)

    axes[2].bar(x, peak_yaw_rate, color=["#94a3b8", "#ef4444", "#2563eb"])
    axes[2].set_xticks(x, labels)
    axes[2].set_title("Peak yaw rate")
    axes[2].set_ylabel("rad/s")
    axes[2].grid(True, axis="y", alpha=0.25)

    fig.suptitle("Archived step-test snapshot already bundled in the repository", y=1.02)
    fig.savefig(FIG_DIR / "uuv_v22_engine_snapshot.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def main() -> None:
    ensure_dirs()
    plot_architecture_overview()
    plot_runtime_step_pipeline()
    plot_6dof_axes()
    plot_6dof_forces_3d()
    plot_vehicle_layout_3d()
    plot_vehicle_layout_top()
    plot_vehicle_layout_side()
    plot_buoyancy_model()
    plot_thruster_model()
    plot_hydrodynamics_comparison()
    plot_runtime_term_matrix()
    plot_custom_wrench_pipeline()
    plot_yaw_pipeline()
    plot_mode_paths_xy()
    plot_mode_metrics()
    plot_engine_snapshot()
    print(f"generated figures in {FIG_DIR}")


if __name__ == "__main__":
    main()

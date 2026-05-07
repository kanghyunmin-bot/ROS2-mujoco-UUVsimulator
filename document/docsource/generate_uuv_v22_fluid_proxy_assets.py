#!/usr/bin/env python3
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches


ROOT = Path(__file__).resolve().parents[2]
DOCSRC_DIR = Path(__file__).resolve().parent
FIG_DIR = DOCSRC_DIR / "figures_v22_latest"
SCENE_PATH = ROOT / "uuv_mujoco" / "v2.2" / "scenes" / "tank_current_scene.xml"


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Apple SD Gothic Neo", "Noto Sans CJK KR", "Arial", "DejaVu Sans"],
        "axes.titlesize": 16,
        "axes.labelsize": 12,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "figure.titlesize": 18,
    }
)


def ensure_dirs() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)


def parse_scene() -> dict[str, dict]:
    root = ET.parse(SCENE_PATH).getroot()
    base = root.find(".//body[@name='base_link']")
    if base is None:
        raise RuntimeError("base_link not found in scene")

    out: dict[str, dict] = {}
    for geom in base.findall("geom"):
        name = geom.attrib.get("name", "")
        if not name.startswith("fluid_"):
            continue
        out[name] = {
            "type": geom.attrib.get("type", ""),
            "pos": [float(v) for v in geom.attrib.get("pos", "0 0 0").split()],
            "size": [float(v) for v in geom.attrib.get("size", "0 0 0").split()],
            "fluidcoef": [float(v) for v in geom.attrib.get("fluidcoef", "0 0 0 0 0").split()],
        }
    return out


def add_ellipse(ax, pos_xy, size_xy, fc, ec, alpha=0.9, lw=2.0, z=2):
    patch = patches.Ellipse(
        pos_xy,
        width=size_xy[0] * 2.0,
        height=size_xy[1] * 2.0,
        facecolor=fc,
        edgecolor=ec,
        linewidth=lw,
        alpha=alpha,
        zorder=z,
    )
    ax.add_patch(patch)


def add_box(ax, center_xy, half_xy, fc, ec, alpha=0.8, lw=1.8, z=3):
    patch = patches.Rectangle(
        (center_xy[0] - half_xy[0], center_xy[1] - half_xy[1]),
        half_xy[0] * 2.0,
        half_xy[1] * 2.0,
        facecolor=fc,
        edgecolor=ec,
        linewidth=lw,
        alpha=alpha,
        zorder=z,
    )
    ax.add_patch(patch)


def style_axes(ax, xlab: str, ylab: str) -> None:
    ax.set_xlabel(xlab)
    ax.set_ylabel(ylab)
    ax.grid(True, alpha=0.18)
    ax.axhline(0.0, color="#cbd5e1", linewidth=1.0, zorder=0)
    ax.axvline(0.0, color="#cbd5e1", linewidth=1.0, zorder=0)
    ax.set_aspect("equal", adjustable="box")


def draw_proxy_overview(geoms: dict[str, dict]) -> None:
    fig = plt.figure(figsize=(14.8, 8.4), dpi=220, constrained_layout=True)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.3, 0.9], height_ratios=[1.0, 1.0])

    ax_top = fig.add_subplot(gs[:, 0])
    ax_note = fig.add_subplot(gs[0, 1])
    ax_side = fig.add_subplot(gs[1, 1])

    center = geoms["fluid_center_enclosure"]
    port = geoms["fluid_port_lower_body"]
    star = geoms["fluid_starboard_lower_body"]
    plates = [
        geoms["fluid_port_top_plate"],
        geoms["fluid_port_bottom_plate"],
        geoms["fluid_starboard_top_plate"],
        geoms["fluid_starboard_bottom_plate"],
    ]

    add_ellipse(ax_top, (center["pos"][0], center["pos"][1]), (center["size"][0], center["size"][1]), "#9bd7ff", "#0f4c81", 0.82)
    add_ellipse(ax_top, (port["pos"][0], port["pos"][1]), (port["size"][0], port["size"][1]), "#8ef0d0", "#0f766e", 0.82)
    add_ellipse(ax_top, (star["pos"][0], star["pos"][1]), (star["size"][0], star["size"][1]), "#8ef0d0", "#0f766e", 0.82)
    for plate in plates:
        add_box(ax_top, (plate["pos"][0], plate["pos"][1]), (plate["size"][0], plate["size"][1]), "#ffd59e", "#b45309", 0.92)

    ax_top.text(center["pos"][0], center["pos"][1], "Center enclosure", ha="center", va="center", fontsize=11, fontweight="bold", color="#0f172a")
    ax_top.text(port["pos"][0], port["pos"][1] + 0.145, "Port lower body", ha="center", va="center", fontsize=10.5, fontweight="bold", color="#0f172a")
    ax_top.text(star["pos"][0], star["pos"][1] - 0.145, "Starboard lower body", ha="center", va="center", fontsize=10.5, fontweight="bold", color="#0f172a")
    ax_top.text(0.38, 0.30, "Top / bottom\nplates", ha="left", va="center", fontsize=10, color="#7c2d12")

    ax_top.set_title("Top View: current fluid proxy layout", pad=10, fontweight="bold")
    style_axes(ax_top, "X [m]", "Y [m]")
    ax_top.set_xlim(-0.60, 0.60)
    ax_top.set_ylim(-0.45, 0.45)

    ax_note.axis("off")
    ax_note.set_title("Role and coefficient meaning", loc="left", fontweight="bold", pad=6)
    note_text = (
        "fluidcoef = [blunt, slender, angular, kutta, magnus]\n\n"
        "Center enclosure\n"
        "  - main volume proxy\n"
        "  - dominant angular resistance\n\n"
        "Port / starboard lower body\n"
        "  - forward drag proxy\n"
        "  - yaw damping via left-right lever arm\n\n"
        "Top / bottom plates\n"
        "  - flat upper / lower area reinforcement\n"
        "  - added mainly for heave-direction drag"
    )
    ax_note.text(
        0.02,
        0.98,
        note_text,
        va="top",
        ha="left",
        fontsize=11.2,
        linespacing=1.45,
        bbox=dict(boxstyle="round,pad=0.55", facecolor="#f8fafc", edgecolor="#cbd5e1"),
    )

    representative = [
        ("fluid_port_lower_body", "#8ef0d0", "#0f766e", "Ellipsoid lower body"),
        ("fluid_port_top_plate", "#ffd59e", "#b45309", "Top plate"),
        ("fluid_port_bottom_plate", "#ffd59e", "#b45309", "Bottom plate"),
    ]
    for key, fc, ec, label in representative:
        geom = geoms[key]
        if geom["type"] == "ellipsoid":
            add_ellipse(ax_side, (geom["pos"][0], geom["pos"][2]), (geom["size"][0], geom["size"][2]), fc, ec, 0.82)
        else:
            add_box(ax_side, (geom["pos"][0], geom["pos"][2]), (geom["size"][0], geom["size"][2]), fc, ec, 0.92)
            ax_side.text(geom["pos"][0] + 0.28, geom["pos"][2], label, fontsize=9.5, color="#7c2d12", va="center")

    ax_side.set_title("Side View: representative lower-body stack", pad=10, fontweight="bold")
    style_axes(ax_side, "X [m]", "Z [m]")
    ax_side.set_xlim(-0.55, 0.55)
    ax_side.set_ylim(-0.18, 0.22)

    fig.suptitle("Ellipsoid Fluid Proxy Design", fontsize=19, fontweight="bold")
    fig.savefig(FIG_DIR / "uuv_v22_latest_fluid_proxy_overview.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def draw_plate_compare(geoms: dict[str, dict]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14.8, 6.6), dpi=220, constrained_layout=True)
    before_ax, after_ax = axes

    lower = geoms["fluid_port_lower_body"]
    top = geoms["fluid_port_top_plate"]
    bottom = geoms["fluid_port_bottom_plate"]

    add_ellipse(before_ax, (lower["pos"][0], lower["pos"][2]), (lower["size"][0], lower["size"][2]), "#8ef0d0", "#0f766e", 0.86, lw=2.2)
    before_ax.text(0.0, 0.0, "rounded upper / lower\narea only", ha="center", va="center", fontsize=12, fontweight="bold", color="#14532d")
    before_ax.set_title("Before: lower-body ellipsoid only", fontweight="bold")
    style_axes(before_ax, "X [m]", "Z [m]")
    before_ax.set_xlim(-0.55, 0.55)
    before_ax.set_ylim(-0.18, 0.22)

    add_ellipse(after_ax, (lower["pos"][0], lower["pos"][2]), (lower["size"][0], lower["size"][2]), "#8ef0d0", "#0f766e", 0.82, lw=2.0)
    add_box(after_ax, (top["pos"][0], top["pos"][2]), (top["size"][0], top["size"][2]), "#ffd59e", "#b45309", 0.95, lw=2.1)
    add_box(after_ax, (bottom["pos"][0], bottom["pos"][2]), (bottom["size"][0], bottom["size"][2]), "#ffd59e", "#b45309", 0.95, lw=2.1)
    after_ax.annotate(
        "top plate",
        xy=(top["pos"][0] + top["size"][0], top["pos"][2]),
        xytext=(0.34, 0.13),
        arrowprops=dict(arrowstyle="->", color="#7c2d12", linewidth=1.6),
        fontsize=10.5,
        color="#7c2d12",
        fontweight="bold",
    )
    after_ax.annotate(
        "bottom plate",
        xy=(bottom["pos"][0] + bottom["size"][0], bottom["pos"][2]),
        xytext=(0.34, -0.09),
        arrowprops=dict(arrowstyle="->", color="#7c2d12", linewidth=1.6),
        fontsize=10.5,
        color="#7c2d12",
        fontweight="bold",
    )
    after_ax.text(
        0.0,
        0.0,
        "flatter upper / lower\nprojected area",
        ha="center",
        va="center",
        fontsize=12,
        fontweight="bold",
        color="#7c2d12",
        bbox=dict(boxstyle="round,pad=0.30", facecolor="white", edgecolor="#fed7aa"),
    )
    after_ax.set_title("After: ellipsoid + thin plate proxies", fontweight="bold")
    style_axes(after_ax, "X [m]", "Z [m]")
    after_ax.set_xlim(-0.55, 0.55)
    after_ax.set_ylim(-0.18, 0.22)

    fig.suptitle("Why Plate Proxy Was Added", fontsize=19, fontweight="bold")
    fig.text(
        0.5,
        0.02,
        "Plate proxies do not add lift terms. They reinforce the flat upper/lower area so heave-direction drag can be represented more realistically.",
        ha="center",
        fontsize=11.5,
        color="#334155",
    )
    fig.savefig(FIG_DIR / "uuv_v22_latest_plate_proxy_compare.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def main() -> None:
    ensure_dirs()
    geoms = parse_scene()
    draw_proxy_overview(geoms)
    draw_plate_compare(geoms)


if __name__ == "__main__":
    main()

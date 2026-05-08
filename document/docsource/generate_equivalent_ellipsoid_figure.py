from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse, FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[2]
DOC_DIR = ROOT / "document"
FIG_DIR = DOC_DIR / "figures"
OUT_PATH = FIG_DIR / "equivalent_ellipsoid_hydrodynamics.png"


A = 0.160  # x semi-axis [m]
B = 0.106  # y semi-axis [m]
C = 0.147  # z semi-axis [m]
RHO = 1000.0


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Apple SD Gothic Neo", "Helvetica", "Arial", "DejaVu Sans"],
        "axes.unicode_minus": False,
    }
)


def add_title(ax, text: str) -> None:
    ax.text(0.02, 0.96, text, transform=ax.transAxes, ha="left", va="top", fontsize=15, fontweight="bold", color="#1e262e")


def panel_background(ax) -> None:
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    bg = FancyBboxPatch(
        (0.01, 0.02),
        0.98,
        0.96,
        boxstyle="round,pad=0.018,rounding_size=0.03",
        linewidth=1.0,
        edgecolor="#d8e0e8",
        facecolor="#f9fbfd",
        transform=ax.transAxes,
    )
    ax.add_patch(bg)


def draw_dimension(ax, start, end, text, text_offset=(0, 0), color="#0f567a"):
    arrow = FancyArrowPatch(start, end, arrowstyle="<->", mutation_scale=12, linewidth=1.6, color=color)
    ax.add_patch(arrow)
    mx = (start[0] + end[0]) / 2 + text_offset[0]
    my = (start[1] + end[1]) / 2 + text_offset[1]
    ax.text(mx, my, text, ha="center", va="center", fontsize=10.5, color=color, fontweight="bold")


def draw_panel_shape(ax) -> None:
    panel_background(ax)
    add_title(ax, "1. Equivalent ellipsoid는 어떤 모양인가?")

    # top view (x-y)
    top_center = (0.30, 0.62)
    top = Ellipse(top_center, width=0.34, height=0.22, facecolor="#dbeafe", edgecolor="#4c78a8", linewidth=2.0)
    ax.add_patch(top)
    ax.text(top_center[0], 0.79, "Top view (x-y)", ha="center", va="center", fontsize=11, color="#4c78a8")
    draw_dimension(ax, (0.13, 0.47), (0.47, 0.47), "2a = 0.320 m", (0, -0.03))
    draw_dimension(ax, (0.50, 0.51), (0.50, 0.73), "2b = 0.212 m", (0.06, 0))

    # side view (x-z)
    side_center = (0.73, 0.62)
    side = Ellipse(side_center, width=0.34, height=0.30, facecolor="#e0f2fe", edgecolor="#0ea5e9", linewidth=2.0)
    ax.add_patch(side)
    ax.text(side_center[0], 0.79, "Side view (x-z)", ha="center", va="center", fontsize=11, color="#0ea5e9")
    draw_dimension(ax, (0.56, 0.44), (0.90, 0.44), "2a = 0.320 m", (0, -0.03), color="#0284c7")
    draw_dimension(ax, (0.93, 0.47), (0.93, 0.77), "2c = 0.294 m", (0.06, 0), color="#0284c7")

    volume = 4.0 / 3.0 * math.pi * A * B * C
    ax.text(
        0.50,
        0.20,
        "실제 ROV 형상을 그대로 유체해석하지 않고,\n"
        "부피와 외형 envelope가 비슷한 대표 타원체로 근사한다.\n"
        f"semi-axes = (a, b, c) = ({A:.3f}, {B:.3f}, {C:.3f}) m,  volume ≈ {volume:.5f} m³",
        ha="center",
        va="center",
        fontsize=11,
        color="#1e262e",
    )


def draw_panel_forces(ax) -> None:
    panel_background(ax)
    add_title(ax, "2. 유체는 이 형상에 어떻게 작용하는가?")

    cx, cy = 0.50, 0.58
    body = Ellipse((cx, cy), width=0.34, height=0.26, facecolor="#eaf4ff", edgecolor="#4c78a8", linewidth=2.0)
    ax.add_patch(body)

    # motion arrows
    arrows = [
        ((0.12, cy), (0.31, cy), "#d97706", "forward velocity v_x"),
        ((cx, 0.20), (cx, 0.39), "#16a34a", "heave velocity v_z"),
        ((0.50, 0.89), (0.50, 0.73), "#dc2626", "yaw / rotation"),
    ]
    for start, end, color, label in arrows:
        ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=16, linewidth=2.2, color=color))
        ax.text(start[0], start[1] + (0.03 if start[1] < end[1] else -0.03), label, fontsize=10.5, color=color, ha="center")

    # drag opposite arrow
    ax.add_patch(FancyArrowPatch((0.69, cy), (0.86, cy), arrowstyle="-|>", mutation_scale=16, linewidth=2.2, color="#1f2937"))
    ax.text(0.86, cy + 0.04, "drag opposes motion", ha="right", va="bottom", fontsize=10.5, color="#1f2937")

    area_x = math.pi * B * C
    area_y = math.pi * A * C
    area_z = math.pi * A * B
    ax.text(
        0.08,
        0.12,
        "Projected area 기준 drag\n"
        f"A_x = πbc ≈ {area_x:.4f} m²\n"
        f"A_y = πac ≈ {area_y:.4f} m²\n"
        f"A_z = πab ≈ {area_z:.4f} m²",
        ha="left",
        va="bottom",
        fontsize=11,
        color="#1e262e",
    )
    ax.text(
        0.57,
        0.11,
        "형상 축 길이로부터 baseline 생성\n"
        "• volume → buoyancy baseline\n"
        "• projected area → drag baseline\n"
        "• semi-axes → added mass baseline",
        ha="left",
        va="bottom",
        fontsize=11,
        color="#1e262e",
    )


def draw_panel_pipeline(ax) -> None:
    panel_background(ax)
    add_title(ax, "3. 지금 시뮬레이터는 이걸 어떻게 쓰는가?")

    # flow boxes
    boxes = [
        (0.07, 0.62, 0.24, 0.18, "#dbeafe", "#1d4ed8", "Equivalent\nellipsoid"),
        (0.38, 0.62, 0.24, 0.18, "#e0f2fe", "#0284c7", "Baseline\ncoefficients"),
        (0.69, 0.62, 0.24, 0.18, "#dcfce7", "#15803d", "Custom 6-DOF\nwrench"),
    ]
    for x, y, w, h, fc, ec, text in boxes:
        patch = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.03", facecolor=fc, edgecolor=ec, linewidth=2.0)
        ax.add_patch(patch)
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=13, color=ec, fontweight="bold")

    for x1, x2 in [(0.31, 0.38), (0.62, 0.69)]:
        ax.add_patch(FancyArrowPatch((x1, 0.71), (x2, 0.71), arrowstyle="-|>", mutation_scale=16, linewidth=2.0, color="#475569"))

    ax.text(
        0.19,
        0.48,
        "input\n(a, b, c)",
        ha="center",
        va="center",
        fontsize=11,
        color="#1e262e",
    )
    ax.text(
        0.50,
        0.48,
        "volume,\nprojected area,\nadded-mass baseline,\ndamping baseline",
        ha="center",
        va="center",
        fontsize=10.8,
        color="#1e262e",
    )
    ax.text(
        0.81,
        0.48,
        "buoyancy + CoB torque\n+ added mass\n+ added-mass Coriolis\n+ linear/quadratic damping",
        ha="center",
        va="center",
        fontsize=10.8,
        color="#1e262e",
    )

    ax.text(
        0.50,
        0.20,
        "중요한 점: MuJoCo가 이 타원체 표면에 유체를 직접 해석하는 것이 아니라,\n"
        "타원체로부터 baseline coefficient를 만든 뒤, 현재 프로젝트의 custom hydrodynamics가\n"
        "최종 힘/토크(wrench)를 계산해서 MuJoCo에 외력으로 주입한다.",
        ha="center",
        va="center",
        fontsize=11.2,
        color="#1e262e",
    )


def main() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(15.5, 6.2), dpi=220, constrained_layout=True)
    gs = fig.add_gridspec(1, 3, wspace=0.06)
    axs = [fig.add_subplot(gs[0, i]) for i in range(3)]

    draw_panel_shape(axs[0])
    draw_panel_forces(axs[1])
    draw_panel_pipeline(axs[2])

    fig.suptitle("Equivalent ellipsoid 기반 hydrodynamics 설명 그림", fontsize=18, fontweight="bold", color="#1e262e")
    fig.savefig(OUT_PATH, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)
    print(f"saved {OUT_PATH}")


if __name__ == "__main__":
    main()

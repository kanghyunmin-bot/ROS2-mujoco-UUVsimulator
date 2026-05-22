from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[2]
DOCSRC_DIR = Path(__file__).resolve().parent
FIG_DIR = DOCSRC_DIR / "figures"
OUT_PATH = FIG_DIR / "pseudo_flow_equivalent_ellipsoid.png"

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


def make_panel_bg(ax) -> None:
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.add_patch(
        FancyBboxPatch(
            (0.01, 0.02),
            0.98,
            0.96,
            boxstyle="round,pad=0.018,rounding_size=0.03",
            linewidth=1.0,
            edgecolor="#d8e0e8",
            facecolor="#f9fbfd",
            transform=ax.transAxes,
        )
    )


def pseudo_flow_field(x: np.ndarray, y: np.ndarray, a: float, b: float, direction: str = "x") -> tuple[np.ndarray, np.ndarray]:
    if direction == "x":
        xn = x / a
        yn = y / b
        r2 = xn * xn + yn * yn
        r2_safe = np.where(r2 < 1.05, 1.05, r2)
        u = 1.0 - (xn * xn - yn * yn) / (r2_safe * r2_safe)
        v = -2.0 * xn * yn / (r2_safe * r2_safe)
    elif direction == "z":
        # Rotate the same pseudo-flow idea by 90 degrees so flow moves bottom->top.
        xn = x / a
        yn = y / b
        r2 = xn * xn + yn * yn
        r2_safe = np.where(r2 < 1.05, 1.05, r2)
        u = -2.0 * xn * yn / (r2_safe * r2_safe)
        v = 1.0 + (xn * xn - yn * yn) / (r2_safe * r2_safe)
    else:
        raise ValueError(direction)

    inside = r2 <= 1.0
    u = np.where(inside, np.nan, u)
    v = np.where(inside, np.nan, v)
    return u, v


def draw_stream_panel(ax, title: str, a: float, b: float, direction: str, x_label: str, y_label: str, ellipse_color: str) -> None:
    make_panel_bg(ax)
    ax.text(0.04, 0.95, title, transform=ax.transAxes, ha="left", va="top", fontsize=14, fontweight="bold", color="#1e262e")
    ax.text(0.04, 0.90, "Pseudo-flow based on current relative-velocity drag model (not CFD)", transform=ax.transAxes,
            ha="left", va="top", fontsize=9.7, color="#617181")

    inner = ax.inset_axes([0.08, 0.12, 0.64, 0.72])
    inner.set_aspect("equal")
    x = np.linspace(-0.32, 0.32, 360)
    y = np.linspace(-0.28, 0.28, 320)
    X, Y = np.meshgrid(x, y)
    U, V = pseudo_flow_field(X, Y, a, b, direction=direction)
    speed = np.sqrt(np.nan_to_num(U) ** 2 + np.nan_to_num(V) ** 2)

    inner.streamplot(
        x,
        y,
        U,
        V,
        density=1.7,
        color=speed,
        cmap="Blues",
        linewidth=1.0,
        arrowsize=0.9,
    )
    inner.add_patch(Ellipse((0, 0), 2 * a, 2 * b, facecolor="#dbeafe", edgecolor=ellipse_color, linewidth=2.4, zorder=5))
    inner.set_xlim(-0.30, 0.30)
    inner.set_ylim(-0.24, 0.24)
    inner.set_xticks([])
    inner.set_yticks([])
    for spine in inner.spines.values():
        spine.set_visible(False)

    # Direction arrows
    if direction == "x":
        inner.add_patch(FancyArrowPatch((-0.28, 0.18), (-0.08, 0.18), arrowstyle="-|>", mutation_scale=14, linewidth=2.0, color="#d97706"))
        inner.text(-0.28, 0.205, "relative flow", color="#d97706", fontsize=10, ha="left")
        inner.add_patch(FancyArrowPatch((0.14, -0.18), (0.28, -0.18), arrowstyle="-|>", mutation_scale=14, linewidth=2.0, color="#1f2937"))
        inner.text(0.28, -0.155, "drag", color="#1f2937", fontsize=10, ha="right")
    else:
        inner.add_patch(FancyArrowPatch((0.22, -0.22), (0.22, -0.04), arrowstyle="-|>", mutation_scale=14, linewidth=2.0, color="#16a34a"))
        inner.text(0.22, -0.245, "relative flow", color="#16a34a", fontsize=10, ha="center")
        inner.add_patch(FancyArrowPatch((-0.22, 0.04), (-0.22, -0.12), arrowstyle="-|>", mutation_scale=14, linewidth=2.0, color="#1f2937"))
        inner.text(-0.22, 0.065, "drag", color="#1f2937", fontsize=10, ha="center")

    side = ax.inset_axes([0.76, 0.18, 0.18, 0.60])
    side.set_xlim(0, 1)
    side.set_ylim(0, 1)
    side.axis("off")
    side.add_patch(FancyBboxPatch((0.02, 0.02), 0.96, 0.96, boxstyle="round,pad=0.02,rounding_size=0.03",
                                  facecolor="white", edgecolor="#d8e0e8"))
    side.text(0.5, 0.92, "Model link", ha="center", va="center", fontsize=10.5, fontweight="bold", color="#0f567a")
    side.text(
        0.08,
        0.80,
        f"{x_label}, {y_label}\nsemi-axes 기준\nstreamline-like view",
        ha="left",
        va="top",
        fontsize=9.7,
        color="#1e262e",
    )
    side.text(
        0.08,
        0.55,
        "현재 모델 의미:\n"
        "• 상대유속 방향 확인\n"
        "• 진행 방향에 수직한 단면적 사용\n"
        "• drag는 motion 반대 방향",
        ha="left",
        va="top",
        fontsize=9.5,
        color="#1e262e",
    )


def draw_summary_panel(ax) -> None:
    make_panel_bg(ax)
    ax.text(0.04, 0.95, "이 그림이 현재 모델과 연결되는 방식", transform=ax.transAxes, ha="left", va="top",
            fontsize=14, fontweight="bold", color="#1e262e")

    area_x = math.pi * B * C
    area_y = math.pi * A * C
    area_z = math.pi * A * B
    volume = 4.0 / 3.0 * math.pi * A * B * C

    boxes = [
        (0.05, 0.63, 0.24, 0.18, "#dbeafe", "#1d4ed8", "Equivalent\nellipsoid"),
        (0.38, 0.63, 0.24, 0.18, "#e0f2fe", "#0284c7", "Projected area /\nvolume"),
        (0.71, 0.63, 0.24, 0.18, "#dcfce7", "#15803d", "Custom 6-DOF\nwrench"),
    ]
    for x, y, w, h, fc, ec, text in boxes:
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.03",
                                    facecolor=fc, edgecolor=ec, linewidth=2.0))
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=13, fontweight="bold", color=ec)

    for x1, x2 in [(0.29, 0.38), (0.62, 0.71)]:
        ax.add_patch(FancyArrowPatch((x1, 0.72), (x2, 0.72), arrowstyle="-|>", mutation_scale=16, linewidth=2.0, color="#475569"))

    ax.text(
        0.08,
        0.46,
        "semi-axes\n"
        f"a={A:.3f}, b={B:.3f}, c={C:.3f} m",
        ha="left",
        va="top",
        fontsize=11,
        color="#1e262e",
    )
    ax.text(
        0.36,
        0.46,
        "projected area\n"
        f"A_x = πbc = {area_x:.4f} m²\n"
        f"A_y = πac = {area_y:.4f} m²\n"
        f"A_z = πab = {area_z:.4f} m²\n"
        f"V = 4/3 πabc = {volume:.5f} m³",
        ha="left",
        va="top",
        fontsize=10.8,
        color="#1e262e",
    )
    ax.text(
        0.68,
        0.46,
        "현재 runtime에서 사용\n"
        "• buoyancy baseline\n"
        "• added mass baseline\n"
        "• damping baseline\n"
        "• xfrc_applied 로 외력 주입",
        ha="left",
        va="top",
        fontsize=10.8,
        color="#1e262e",
    )
    ax.text(
        0.50,
        0.16,
        "즉 이 그림은 CFD 결과가 아니라,\n"
        "현재 simulator의 relative-flow / projected-area / coefficient-based hydrodynamics를\n"
        "유선처럼 직관적으로 보이게 만든 pseudo-flow 설명 그림이다.",
        ha="center",
        va="center",
        fontsize=11.2,
        color="#1e262e",
    )


def main() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(15.5, 11.0), dpi=220, constrained_layout=True)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 0.88], wspace=0.05, hspace=0.08)

    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, :])

    draw_stream_panel(ax1, "A. Surge 기준 pseudo-flow", A, B, "x", "x", "y", "#2563eb")
    draw_stream_panel(ax2, "B. Heave 기준 pseudo-flow", A, C, "z", "x", "z", "#0ea5e9")
    draw_summary_panel(ax3)

    fig.suptitle("Pseudo-flow visualization for the current equivalent-ellipsoid hydrodynamics model", fontsize=18,
                 fontweight="bold", color="#1e262e")
    fig.savefig(OUT_PATH, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)
    print(f"saved {OUT_PATH}")


if __name__ == "__main__":
    main()

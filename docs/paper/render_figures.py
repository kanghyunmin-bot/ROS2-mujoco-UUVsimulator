"""Render paper figures from archived 10 Hz traces and RMSE tables."""

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
import numpy as np


ROOT = Path(__file__).resolve().parent
BLUE = "#143F78"
RED = "#BA243C"
ORANGE = "#D57B21"
INK = "#111111"
GRAY = "#5A6370"
GRID = "#DDE2E8"
DASHED = (0, (4, 2))
MEASURED = ("measured", "Measured", INK, "-", None)
AXES = (
    ("surge", "Surge velocity", (-.3, 1.6)),
    ("sway", "Sway velocity", (-.7, 1.0)),
    ("heave", "Relative depth", (-600, 300)),
    ("roll", "Roll", (-160, 60)),
    ("pitch", "Pitch", (-40, 70)),
    ("yaw", "Accumulated yaw", (-1200, 150)),
)
FIGURES = (
    (
        "figure-01-initial-final",
        (
            ("initial", "Initial", BLUE, DASHED, "initial"),
            ("final", "Final", RED, "-", "final_yaw_tuned"),
        ),
        "Final is the experimental yaw-tuned profile. This trial was used for fitting.",
    ),
    (
        "figure-02-simulator-comparison",
        (
            ("stonefish", "Stonefish", BLUE, DASHED, "stonefish"),
            ("gazebo", "Gazebo", ORANGE, (0, (4.5, 1.8, 1.1, 1.8)), "gazebo"),
            ("uuv_sim", "UUV-Sim", RED, "-", "uuv_sim"),
        ),
        "UUV-Sim uses the experimental yaw fit. This trial was used for fitting.",
    ),
)


def draw_figure(stem, series, note):
    data = np.genfromtxt(ROOT / "data" / f"{stem}-timeseries.csv", delimiter=",", names=True)
    with (ROOT / "data" / f"{stem}-rmse.csv").open(newline="") as stream:
        rows = {row["axis"]: row for row in csv.DictReader(stream)}

    fig = plt.figure(figsize=(178 / 25.4, 142 / 25.4), facecolor="white")
    grid = fig.add_gridspec(
        3, 2, left=.065, right=.989, top=.901, bottom=.135,
        hspace=.68, wspace=.20,
    )
    time = data["t_s"] - data["t_s"][0]
    for index, (axis, axis_title, limits) in enumerate(AXES):
        line_ax, bar_ax = grid[index // 2, index % 2].subgridspec(
            1, 2, width_ratios=(3.15, 1.35), wspace=.25,
        ).subplots()
        row = rows[axis]
        unit = row["unit"]
        valid = data[f"{axis}_valid"].astype(bool)
        assert valid.sum() == int(row["paired_samples"])
        measured = data[f"{axis}_measured"]
        line_ax.plot(time, np.where(valid, measured, np.nan), color=INK, lw=.76, zorder=5)

        errors = []
        for name, _, color, linestyle, metric_key in series:
            signal = data[f"{axis}_{name}"]
            line_ax.plot(time, signal, color=color, ls=linestyle, lw=.72, zorder=4)
            score = float(row[metric_key])
            np.testing.assert_allclose(
                np.sqrt(np.mean((signal[valid] - measured[valid]) ** 2)),
                score, atol=2e-8, rtol=0,
            )
            errors.append(score)

        line_ax.set_title(f"({chr(97 + index)}) {axis_title}", loc="left", fontsize=8,
                          fontweight="bold", pad=5)
        line_ax.set_ylabel(unit, fontsize=7, labelpad=2)
        if axis == "heave" and stem == "figure-01-initial-final":
            limits = (-220, 280)
        if axis == "roll" and stem == "figure-02-simulator-comparison":
            limits = (-80, 60)
        line_ax.set(xlim=(0, time[-1]), ylim=limits)
        line_ax.set_xticks([0, 50, 100, 150, 200])
        if index >= 4:
            line_ax.set_xlabel("Replay time (s)", fontsize=7, labelpad=2)

        bar_ax.set_title("RMSE", fontsize=7, pad=5)
        positions = np.arange(len(series))
        bars = bar_ax.bar(positions, errors, width=.62, color=[entry[2] for entry in series], zorder=3)
        abbreviations = ["I", "F"] if len(series) == 2 else ["S", "G", "U"]
        bar_ax.set_xticks(positions, abbreviations)
        bar_ax.tick_params(axis="x", length=0, pad=3)
        bar_ax.set_xlim(-.55, len(series) - .45)
        bar_ax.set_ylim(0, max(errors) * 1.32)
        bar_ax.bar_label(bars, fmt="%.3f" if unit == "m/s" else "%.1f",
                         padding=2, fontsize=6.0)
        for ax in (line_ax, bar_ax):
            ax.spines[["top", "right"]].set_visible(False)
            ax.set_axisbelow(True)
            ax.grid(axis="y", color=GRID, linewidth=.5)
            ax.yaxis.set_major_locator(MaxNLocator(4))
            ax.tick_params(labelsize=6.2, length=2.4, width=.6, pad=2)

    legend = [
        Line2D([], [], color=color, ls=linestyle, lw=1.15, label=label)
        for _, label, color, linestyle, _ in (MEASURED, *series)
    ]
    fig.legend(handles=legend, loc="upper center", bbox_to_anchor=(.5, .975),
               frameon=False, ncol=len(legend), fontsize=7.6, handlelength=2.5,
               columnspacing=1.7)
    abbreviations = "I  Initial   ·   F  Final" if len(series) == 2 else (
        "S  Stonefish   ·   G  Gazebo   ·   U  UUV-Sim"
    )
    fig.text(.5, .079, abbreviations, ha="center", va="center", fontsize=6.4, color=GRAY)
    context = "DFKI random path · Paired 10 Hz samples"
    if len(series) == 3:
        context += " · Baseline actuator and tank settings differ"
    fig.text(.5, .047, context, ha="center", va="center", fontsize=6.4, color=GRAY)
    fig.text(.5, .021, note, ha="center", va="center", fontsize=6.2, color=GRAY)
    for ext in ("pdf", "png"):
        fig.savefig(ROOT / f"{stem}.{ext}", dpi=450)
    plt.close(fig)


if __name__ == "__main__":
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 7, "pdf.fonttype": 42,
        "text.color": INK, "axes.labelcolor": INK, "axes.edgecolor": "#354052",
        "axes.linewidth": .7, "xtick.color": GRAY, "ytick.color": GRAY,
    })
    for spec in FIGURES:
        draw_figure(*spec)

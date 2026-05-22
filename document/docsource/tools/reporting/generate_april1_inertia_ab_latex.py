#!/usr/bin/env python3
from __future__ import annotations

import json
import textwrap
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


DOCSRC = Path(__file__).resolve().parent
BASE_DIR = DOCSRC / "real_bag_2026_04_01_replay"
AB_DIR = DOCSRC / "real_bag_2026_04_01_replay_ab_inertia"
FIG_DIR = AB_DIR / "latex_figures"
TEX_PATH = DOCSRC / "report_sources" / "april1_rosbag" / "real_bag_2026_04_01_inertia_ab_report_kr.tex"

MAIN_BAGS = ("bag_2026-04-01_20-08-11", "bag_2026-04-01_20-20-30")


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def bag_short(name: str) -> str:
    return name.replace("bag_2026-04-01_", "")


def save(fig, name: str) -> str:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    path = FIG_DIR / name
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return f"real_bag_2026_04_01_replay_ab_inertia/latex_figures/{name}"


def metric(result: dict[str, Any], group: str, axis: str | None, key: str) -> float:
    block = result[group] if axis is None else result[group][axis]
    if key == "gain":
        return float(block.get("gain_fit_real_from_sim", {}).get("gain", np.nan))
    return float(block.get(key, np.nan))


def result_pair(base: dict[str, Any], ab: dict[str, Any], bag: str) -> tuple[dict[str, Any], dict[str, Any]]:
    baseline = base["bags"][bag]["replays"]["current_rc_override"]
    inertia = ab["bags"][bag]["replays"]["current_inertia1_rc_override"]
    return baseline, inertia


def plot_pipeline() -> str:
    fig = plt.figure(figsize=(12.5, 3.8))
    ax = fig.add_subplot(111)
    ax.axis("off")
    boxes = [
        ("fixed replay\\nmeasurement", 0.10, "#dce8ff"),
        ("baseline\\nIx,Iy,Iz=1e-6", 0.34, "#fff1cc"),
        ("A/B-1 only\\nIx,Iy,Iz=0.473,0.370,0.739", 0.60, "#e8f4e4"),
        ("compare\\nRMSE corr gain", 0.86, "#eeeeee"),
    ]
    for label, x, color in boxes:
        ax.text(
            x,
            0.55,
            label,
            ha="center",
            va="center",
            fontsize=12,
            bbox=dict(boxstyle="round,pad=0.45", facecolor=color, edgecolor="#555555", linewidth=1.1),
            transform=ax.transAxes,
        )
    for i in range(len(boxes) - 1):
        ax.annotate(
            "",
            xy=(boxes[i + 1][1] - 0.10, 0.55),
            xytext=(boxes[i][1] + 0.10, 0.55),
            arrowprops=dict(arrowstyle="->", lw=1.8, color="#333333"),
            xycoords=ax.transAxes,
        )
    ax.text(
        0.5,
        0.18,
        "Only body_inertia_scale_xyz changed. Thruster gain, allocation, buoyancy, damping, command replay, and sensor correction are unchanged.",
        ha="center",
        va="center",
        fontsize=10.5,
        color="#444444",
        transform=ax.transAxes,
    )
    return save(fig, "inertia_ab_pipeline.png")


def plot_metrics(base: dict[str, Any], ab: dict[str, Any]) -> str:
    panels = [
        ("DVL x RMSE [m/s]", "dvl_velocity_metrics", "x", "rmse"),
        ("DVL y RMSE [m/s]", "dvl_velocity_metrics", "y", "rmse"),
        ("gyro z RMSE [rad/s]", "imu_gyro_metrics", "z", "rmse"),
        ("depth RMSE [m]", "depth_metrics", None, "rmse"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 7.8))
    axes = axes.reshape(-1)
    labels = [bag_short(bag) for bag in MAIN_BAGS]
    x = np.arange(len(labels))
    width = 0.34
    for ax, (title, group, axis, key) in zip(axes, panels):
        baseline_vals = []
        inertia_vals = []
        for bag in MAIN_BAGS:
            baseline, inertia = result_pair(base, ab, bag)
            baseline_vals.append(metric(baseline, group, axis, key))
            inertia_vals.append(metric(inertia, group, axis, key))
        ax.bar(x - width / 2, baseline_vals, width=width, label="baseline", color="#2f6fdd")
        ax.bar(x + width / 2, inertia_vals, width=width, label="inertia-only", color="#2d9b66")
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25)
    axes[0].legend(ncol=2)
    return save(fig, "inertia_ab_rmse.png")


def plot_corr_gain(base: dict[str, Any], ab: dict[str, Any]) -> str:
    metrics = [
        ("DVL x", "dvl_velocity_metrics", "x"),
        ("DVL y", "dvl_velocity_metrics", "y"),
        ("gyro z", "imu_gyro_metrics", "z"),
    ]
    fig, axes = plt.subplots(2, len(MAIN_BAGS), figsize=(13.2, 6.6), sharey="row")
    case_names = ("baseline", "inertia-only")
    for col, bag in enumerate(MAIN_BAGS):
        baseline, inertia = result_pair(base, ab, bag)
        rows = (baseline, inertia)
        corr = np.array([[metric(row, group, axis, "correlation") for _, group, axis in metrics] for row in rows])
        gain = np.array([[metric(row, group, axis, "gain") for _, group, axis in metrics] for row in rows])
        im0 = axes[0, col].imshow(corr, vmin=-1, vmax=1, cmap="coolwarm")
        axes[0, col].set_title(f"{bag_short(bag)} corr")
        axes[0, col].set_xticks(np.arange(len(metrics)))
        axes[0, col].set_xticklabels([m[0] for m in metrics])
        axes[0, col].set_yticks(np.arange(len(case_names)))
        axes[0, col].set_yticklabels(case_names)
        for i in range(len(case_names)):
            for j in range(len(metrics)):
                axes[0, col].text(j, i, f"{corr[i, j]:.2f}", ha="center", va="center", fontsize=10)
        im1 = axes[1, col].imshow(gain, vmin=0, vmax=0.5, cmap="viridis")
        axes[1, col].set_title(f"{bag_short(bag)} real/sim gain")
        axes[1, col].set_xticks(np.arange(len(metrics)))
        axes[1, col].set_xticklabels([m[0] for m in metrics])
        axes[1, col].set_yticks(np.arange(len(case_names)))
        axes[1, col].set_yticklabels(case_names)
        for i in range(len(case_names)):
            for j in range(len(metrics)):
                axes[1, col].text(j, i, f"{gain[i, j]:.2f}", ha="center", va="center", fontsize=10, color="white")
    fig.colorbar(im0, ax=axes[0, :], shrink=0.82, label="signed correlation")
    fig.colorbar(im1, ax=axes[1, :], shrink=0.82, label="gain")
    return save(fig, "inertia_ab_corr_gain.png")


def plot_overlay() -> str:
    fig, axes = plt.subplots(3, len(MAIN_BAGS), figsize=(14.0, 8.0), sharex="col")
    for col, bag in enumerate(MAIN_BAGS):
        base_npz = np.load(BASE_DIR / bag / "current_rc_override_timeseries.npz")
        inertia_npz = np.load(AB_DIR / bag / "current_inertia1_rc_override_timeseries.npz")
        t0 = base_npz["t"]
        t1 = inertia_npz["t"]
        axes[0, col].plot(t0, base_npz["dvl_vel"][:, 0], color="#2f6fdd", lw=0.8, label="baseline DVL x")
        axes[0, col].plot(t1, inertia_npz["dvl_vel"][:, 0], color="#2d9b66", lw=0.8, label="inertia-only DVL x")
        axes[0, col].set_title(f"{bag_short(bag)} DVL x")
        axes[0, col].set_ylabel("m/s")
        axes[0, col].grid(alpha=0.25)
        axes[1, col].plot(t0, base_npz["dvl_vel"][:, 1], color="#2f6fdd", lw=0.8, label="baseline DVL y")
        axes[1, col].plot(t1, inertia_npz["dvl_vel"][:, 1], color="#2d9b66", lw=0.8, label="inertia-only DVL y")
        axes[1, col].set_title(f"{bag_short(bag)} DVL y")
        axes[1, col].set_ylabel("m/s")
        axes[1, col].grid(alpha=0.25)
        axes[2, col].plot(t0, base_npz["imu_gyro"][:, 2], color="#2f6fdd", lw=0.8, label="baseline gyro z")
        axes[2, col].plot(t1, inertia_npz["imu_gyro"][:, 2], color="#2d9b66", lw=0.8, label="inertia-only gyro z")
        axes[2, col].set_title(f"{bag_short(bag)} yaw rate")
        axes[2, col].set_ylabel("rad/s")
        axes[2, col].set_xlabel("bag time [s]")
        axes[2, col].grid(alpha=0.25)
        if col == 0:
            for ax in axes[:, col]:
                ax.legend(fontsize=8)
    return save(fig, "inertia_ab_overlay.png")


def write_tex(figs: dict[str, str]) -> None:
    tex = textwrap.dedent(
        r"""
        \documentclass[11pt,a4paper]{{article}}
        \usepackage{{fontspec}}
        \usepackage[margin=18mm]{{geometry}}
        \usepackage{{graphicx}}
        \usepackage{{booktabs}}
        \usepackage{{tabularx}}
        \usepackage{{array}}
        \usepackage{{float}}
        \usepackage{{caption}}
        \usepackage{{hyperref}}
        \usepackage{{xcolor}}

        \defaultfontfeatures{{Ligatures=TeX,Scale=MatchLowercase}}
        \IfFontExistsTF{{Noto Sans CJK KR}}{{\setmainfont{{Noto Sans CJK KR}}\setsansfont{{Noto Sans CJK KR}}}}{{\setmainfont{{Apple SD Gothic Neo}}\setsansfont{{Apple SD Gothic Neo}}}}
        \IfFontExistsTF{{DejaVu Sans Mono}}{{\setmonofont{{DejaVu Sans Mono}}}}{{\setmonofont{{Menlo}}}}
        \XeTeXlinebreaklocale "ko"
        \XeTeXlinebreakskip = 0pt plus 1pt
        \hypersetup{{colorlinks=true,linkcolor=blue!50!black,urlcolor=blue!50!black}}
        \setlength{{\parskip}}{{0.45em}}
        \setlength{{\parindent}}{{0pt}}
        \renewcommand{{\arraystretch}}{{1.16}}
        \newcolumntype{{Y}}{{>{{\raggedright\arraybackslash}}X}}
        \newcommand{{\code}}[1]{{\texttt{{#1}}}}
        \captionsetup{{font=small,labelfont=bf,justification=raggedright,singlelinecheck=false}}

        \title{{2026-04-01 UUV MuJoCo Inertia A/B 분석\\\large A/B-1: body inertia only}}
        \author{{}}
        \date{{2026-04-28}}

        \begin{{document}}
        \maketitle

        \section{{실험 조건}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{{@@PIPELINE@@}}
          \caption{{A/B-1 구성. baseline은 current profile의 \code{{body\_inertia\_scale\_xyz=[1e-6,1e-6,1e-6]}}, inertia-only는 같은 profile에서 관성 scale만 \code{{[1,1,1]}}로 바꾼다. 다른 물리/추력/센서/명령 replay 조건은 그대로 둔다.}}
        \end{{figure}}

        \section{{정량 비교}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{{@@RMSE@@}}
          \caption{{baseline과 inertia-only의 RMSE 비교. 20:20 depth RMSE는 줄지만, DVL/gyro의 상관과 real/sim gain 개선은 제한적이다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{{@@CORR_GAIN@@}}
          \caption{{상관계수와 real/sim gain. 관성 정상화만으로 yaw rate gain이 1에 가까워지지 않는다. 즉 yaw 과응답의 1차 원인은 관성만이 아니라 thruster force/yaw torque scale 쪽일 가능성이 크다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{{@@OVERLAY@@}}
          \caption{{sim sensor time-series overlay. inertia-only는 일부 peak를 완화하지만 명령-응답 형태 자체를 일관되게 개선하지 않는다.}}
        \end{{figure}}

        \section{{판단}}

        \begin{{table}}[H]
          \centering
          \caption{{핵심 수치}}
          \begin{{tabularx}}{{\linewidth}}{{lYYYY}}
            \toprule
            Bag & Case & DVL x gain & DVL y gain & gyro z gain \\
            \midrule
            20:08 & baseline & 0.254 & 0.150 & 0.351 \\
            20:08 & inertia-only & 0.245 & 0.144 & 0.358 \\
            20:20 & baseline & 0.082 & 0.103 & 0.157 \\
            20:20 & inertia-only & 0.027 & 0.097 & 0.134 \\
            \bottomrule
          \end{{tabularx}}
        \end{{table}}

        결론: \code{{body\_inertia\_scale\_xyz=[1,1,1]}}는 물리적으로 더 타당하지만, 이 변경 하나만으로 실제와 simulation amplitude gap을 해결하지 못한다. 다음 A/B는 \code{{gain\_scale\_all}} 또는 yaw torque/thruster gain만 낮추는 실험이어야 한다. 특히 yaw z gain이 0.35 또는 0.16 수준에 머무르므로 sim yaw response가 여전히 실제보다 크다.

        \end{{document}}
        """
    ).strip()
    tex = tex.replace("{{", "{").replace("}}", "}")
    for key, value in {
        "@@PIPELINE@@": figs["pipeline"],
        "@@RMSE@@": figs["rmse"],
        "@@CORR_GAIN@@": figs["corr_gain"],
        "@@OVERLAY@@": figs["overlay"],
    }.items():
        tex = tex.replace(key, value)
    TEX_PATH.write_text(tex)


def main() -> None:
    TEX_PATH.parent.mkdir(parents=True, exist_ok=True)
    base = load_json(BASE_DIR / "summary.json")
    ab = load_json(AB_DIR / "summary.json")
    figs = {
        "pipeline": plot_pipeline(),
        "rmse": plot_metrics(base, ab),
        "corr_gain": plot_corr_gain(base, ab),
        "overlay": plot_overlay(),
    }
    write_tex(figs)
    print(TEX_PATH)
    for path in figs.values():
        print(DOCSRC / path)


if __name__ == "__main__":
    main()

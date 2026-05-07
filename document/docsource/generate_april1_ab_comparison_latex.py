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
INERTIA_DIR = DOCSRC / "real_bag_2026_04_01_replay_ab_inertia"
GAIN_DIR = DOCSRC / "real_bag_2026_04_01_replay_ab_thruster_gain"
PATCH_DIR = DOCSRC / "real_bag_2026_04_01_replay_after_core_patch"
OUT_DIR = DOCSRC / "real_bag_2026_04_01_ab_comparison"
FIG_DIR = OUT_DIR / "latex_figures"
TEX_PATH = DOCSRC / "real_bag_2026_04_01_ab_comparison_report_kr.tex"

MAIN_BAGS = ("bag_2026-04-01_20-08-11", "bag_2026-04-01_20-20-30")
CASES = ("baseline", "inertia-only", "thruster-gain-only", "patched-current")
CASE_COLORS = {
    "baseline": "#2f6fdd",
    "inertia-only": "#2d9b66",
    "thruster-gain-only": "#c55338",
    "patched-current": "#6f4cc3",
}


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
    return f"real_bag_2026_04_01_ab_comparison/latex_figures/{name}"


def load_results() -> dict[str, dict[str, dict[str, Any]]]:
    base = load_json(BASE_DIR / "summary.json")
    inertia = load_json(INERTIA_DIR / "summary.json")
    gain = load_json(GAIN_DIR / "summary.json")
    patch = load_json(PATCH_DIR / "summary.json")
    out: dict[str, dict[str, dict[str, Any]]] = {}
    for bag in MAIN_BAGS:
        out[bag] = {
            "baseline": base["bags"][bag]["replays"]["current_rc_override"],
            "inertia-only": inertia["bags"][bag]["replays"]["current_inertia1_rc_override"],
            "thruster-gain-only": gain["bags"][bag]["replays"]["current_thruster_gain1_rc_override"],
            "patched-current": patch["bags"][bag]["replays"]["current_rc_override"],
        }
    return out


def metric(result: dict[str, Any], group: str, axis: str | None, key: str) -> float:
    block = result[group] if axis is None else result[group][axis]
    if key == "gain":
        return float(block.get("gain_fit_real_from_sim", {}).get("gain", np.nan))
    return float(block.get(key, np.nan))


def yaw_thruster_p95(result: dict[str, Any]) -> float:
    values = []
    for name, block in result.get("thruster_summary", {}).items():
        if name.startswith("yaw_"):
            value = block.get("force_n", {}).get("p95")
            if value is not None:
                values.append(float(value))
    return max(values) if values else np.nan


def plot_pipeline() -> str:
    fig = plt.figure(figsize=(13.5, 3.8))
    ax = fig.add_subplot(111)
    ax.axis("off")
    boxes = [
        ("corrected\\nreplay baseline", 0.10, "#dce8ff"),
        ("A/B-1\\ninertia only", 0.35, "#e8f4e4"),
        ("A/B-2\\nthruster gain only", 0.62, "#ffe4dc"),
        ("decision\\nnext patch target", 0.88, "#eeeeee"),
    ]
    for text, x, color in boxes:
        ax.text(
            x,
            0.56,
            text,
            ha="center",
            va="center",
            fontsize=12,
            bbox=dict(boxstyle="round,pad=0.45", facecolor=color, edgecolor="#555555", linewidth=1.1),
            transform=ax.transAxes,
        )
    for i in range(len(boxes) - 1):
        ax.annotate(
            "",
            xy=(boxes[i + 1][1] - 0.10, 0.56),
            xytext=(boxes[i][1] + 0.10, 0.56),
            arrowprops=dict(arrowstyle="->", lw=1.8, color="#333333"),
            xycoords=ax.transAxes,
        )
    ax.text(
        0.5,
        0.17,
        "A/B replays change one variable at a time. The final patched-current run uses the persisted uuv_mujoco inertia+thruster gain edits.",
        ha="center",
        va="center",
        fontsize=10.5,
        color="#444444",
        transform=ax.transAxes,
    )
    return save(fig, "ab_pipeline.png")


def plot_rmse(results: dict[str, dict[str, dict[str, Any]]]) -> str:
    panels = [
        ("DVL x RMSE [m/s]", "dvl_velocity_metrics", "x"),
        ("DVL y RMSE [m/s]", "dvl_velocity_metrics", "y"),
        ("gyro z RMSE [rad/s]", "imu_gyro_metrics", "z"),
        ("depth RMSE [m]", "depth_metrics", None),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(13.0, 8.0))
    axes = axes.reshape(-1)
    labels = [bag_short(b) for b in MAIN_BAGS]
    x = np.arange(len(labels))
    width = 0.18
    for ax, (title, group, axis) in zip(axes, panels):
        for ci, case in enumerate(CASES):
            values = [metric(results[bag][case], group, axis, "rmse") for bag in MAIN_BAGS]
            ax.bar(x + (ci - 1.5) * width, values, width=width, label=case, color=CASE_COLORS[case])
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25)
    axes[0].legend(ncol=3, fontsize=8)
    return save(fig, "ab_rmse.png")


def plot_corr_gain(results: dict[str, dict[str, dict[str, Any]]]) -> str:
    metrics = [
        ("DVL x", "dvl_velocity_metrics", "x"),
        ("DVL y", "dvl_velocity_metrics", "y"),
        ("gyro z", "imu_gyro_metrics", "z"),
    ]
    fig, axes = plt.subplots(2, len(MAIN_BAGS), figsize=(13.4, 7.0), sharey="row")
    for col, bag in enumerate(MAIN_BAGS):
        corr = np.array([[metric(results[bag][case], group, axis, "correlation") for _, group, axis in metrics] for case in CASES])
        gain = np.array([[metric(results[bag][case], group, axis, "gain") for _, group, axis in metrics] for case in CASES])
        im0 = axes[0, col].imshow(corr, vmin=-1, vmax=1, cmap="coolwarm")
        axes[0, col].set_title(f"{bag_short(bag)} correlation")
        axes[0, col].set_xticks(np.arange(len(metrics)))
        axes[0, col].set_xticklabels([item[0] for item in metrics])
        axes[0, col].set_yticks(np.arange(len(CASES)))
        axes[0, col].set_yticklabels(CASES)
        for i in range(len(CASES)):
            for j in range(len(metrics)):
                axes[0, col].text(j, i, f"{corr[i, j]:.2f}", ha="center", va="center", fontsize=9)
        im1 = axes[1, col].imshow(gain, vmin=0, vmax=0.65, cmap="viridis")
        axes[1, col].set_title(f"{bag_short(bag)} real/sim gain")
        axes[1, col].set_xticks(np.arange(len(metrics)))
        axes[1, col].set_xticklabels([item[0] for item in metrics])
        axes[1, col].set_yticks(np.arange(len(CASES)))
        axes[1, col].set_yticklabels(CASES)
        for i in range(len(CASES)):
            for j in range(len(metrics)):
                axes[1, col].text(j, i, f"{gain[i, j]:.2f}", ha="center", va="center", fontsize=9, color="white")
    fig.colorbar(im0, ax=axes[0, :], shrink=0.82, label="signed correlation")
    fig.colorbar(im1, ax=axes[1, :], shrink=0.82, label="gain")
    return save(fig, "ab_corr_gain.png")


def plot_thruster_contact(results: dict[str, dict[str, dict[str, Any]]]) -> str:
    labels = [bag_short(b) for b in MAIN_BAGS]
    x = np.arange(len(labels))
    width = 0.18
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.8))
    for ci, case in enumerate(CASES):
        yaw = [yaw_thruster_p95(results[bag][case]) for bag in MAIN_BAGS]
        contact = [float(results[bag][case]["physics_summary"].get("contact_fraction", np.nan)) for bag in MAIN_BAGS]
        axes[0].bar(x + (ci - 1.5) * width, yaw, width=width, label=case, color=CASE_COLORS[case])
        axes[1].bar(x + (ci - 1.5) * width, contact, width=width, label=case, color=CASE_COLORS[case])
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels)
    axes[0].set_title("Yaw thruster force p95")
    axes[0].set_ylabel("N")
    axes[0].grid(axis="y", alpha=0.25)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels)
    axes[1].set_title("Contact fraction")
    axes[1].set_ylabel("fraction")
    axes[1].grid(axis="y", alpha=0.25)
    axes[0].legend(ncol=3, fontsize=8)
    return save(fig, "ab_thruster_contact.png")


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
        \emergencystretch=2em
        \renewcommand{{\arraystretch}}{{1.16}}
        \newcolumntype{{Y}}{{>{{\raggedright\arraybackslash}}X}}
        \newcommand{{\code}}[1]{{\texttt{{#1}}}}
        \captionsetup{{font=small,labelfont=bf,justification=raggedright,singlelinecheck=false}}

        \title{{2026-04-01 UUV MuJoCo A/B 원인분리 분석\\\large inertia-only vs thruster-gain-only}}
        \author{{}}
        \date{{2026-04-28}}

        \begin{{document}}
        \maketitle

        \section{{A/B 구성}}
        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{{@@PIPELINE@@}}
          \caption{{baseline, inertia-only, thruster-gain-only, patched-current 비교 구조. 비교 파이프라인과 sensor frame correction은 고정했다. patched-current는 실제 \code{uuv\_mujoco} config 수정 후 재실행한 결과다.}}
        \end{{figure}}

        \section{{결과 그래프}}
        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{{@@RMSE@@}}
          \caption{{RMSE 비교. thruster-gain-only는 DVL/gyro/depth RMSE를 가장 크게 줄인다. inertia-only는 일부 RMSE를 낮추지만 상관/gain 개선이 일관되지 않다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{{@@CORR_GAIN@@}}
          \caption{{상관계수와 real/sim gain. \code{{gain\_scale\_all=1.0}}은 yaw/DVL amplitude gap을 줄이는 방향으로 작동하지만, 아직 gain 1.0까지는 부족하다. patched-current는 combined override와 거의 같은 결과다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.94\linewidth]{{@@THRUSTER_CONTACT@@}}
          \caption{{yaw thruster force p95와 contact fraction. thruster gain 수정 후 yaw p95 force는 약 149 N에서 51 N 수준으로 낮아진다. contact fraction은 여전히 높아서 depth 평가는 controller-in-loop로 다시 해야 한다.}}
        \end{{figure}}

        \section{{판단}}
        \begin{{table}}[H]
          \centering
          \caption{{핵심 gain 비교}}
          \begin{{tabularx}}{{\linewidth}}{{lYYYY}}
            \toprule
            Bag/Case & DVL x gain & DVL y gain & gyro z gain & 해석 \\
            \midrule
            20:08 baseline & 0.254 & 0.150 & 0.351 & sim amplitude 과대 \\
            20:08 inertia-only & 0.245 & 0.144 & 0.358 & 관성만으로 개선 안 됨 \\
            20:08 thruster-gain-only & 0.377 & 0.211 & 0.573 & 가장 큰 개선 \\
            20:08 patched-current & 0.376 & 0.212 & 0.608 & 실제 config 반영 후 결과 \\
            20:20 baseline & 0.082 & 0.103 & 0.157 & sim amplitude 매우 과대 \\
            20:20 inertia-only & 0.027 & 0.097 & 0.134 & 오히려 일부 악화 \\
            20:20 thruster-gain-only & 0.119 & 0.145 & 0.194 & 개선되나 아직 부족 \\
            20:20 patched-current & 0.130 & 0.197 & 0.203 & 실제 config 반영 후 결과 \\
            \bottomrule
          \end{{tabularx}}
        \end{{table}}

        결론: 현재 핵심 문제는 \code{{body\_inertia\_scale\_xyz}}보다 \code{{gain\_scale\_all=2.9}} 쪽 신호가 더 강하다. 실제 patch는 \code{{gain\_scale\_all=1.0}}과 \code{{body\_inertia\_scale\_xyz=[1,1,1]}}를 반영했다. 관성 정상화는 물리적으로 필요하지만 단독으로 bag fit을 해결하지 못했고, 추력 gain 수정이 RMSE/gain 개선의 주요 원인이다.

        \end{{document}}
        """
    ).strip()
    tex = tex.replace("{{", "{").replace("}}", "}")
    for key, value in {
        "@@PIPELINE@@": figs["pipeline"],
        "@@RMSE@@": figs["rmse"],
        "@@CORR_GAIN@@": figs["corr_gain"],
        "@@THRUSTER_CONTACT@@": figs["thruster_contact"],
    }.items():
        tex = tex.replace(key, value)
    TEX_PATH.write_text(tex)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    results = load_results()
    figs = {
        "pipeline": plot_pipeline(),
        "rmse": plot_rmse(results),
        "corr_gain": plot_corr_gain(results),
        "thruster_contact": plot_thruster_contact(results),
    }
    write_tex(figs)
    print(TEX_PATH)
    for path in figs.values():
        print(DOCSRC / path)


if __name__ == "__main__":
    main()

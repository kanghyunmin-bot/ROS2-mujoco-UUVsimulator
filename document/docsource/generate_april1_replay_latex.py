#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import textwrap
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


DOCSRC = Path(__file__).resolve().parent
REPLAY_DIR = DOCSRC / "real_bag_2026_04_01_replay"
CMD_DIR = DOCSRC / "real_bag_2026_04_01_command_response"
FIG_DIR = REPLAY_DIR / "latex_figures"
TEX_PATH = DOCSRC / "real_bag_2026_04_01_replay_report_kr.tex"

BAGS = ("bag_2026-04-01_20-06-09", "bag_2026-04-01_20-08-11", "bag_2026-04-01_20-20-30")
MAIN_BAGS = ("bag_2026-04-01_20-08-11", "bag_2026-04-01_20-20-30")
CONFIGS = ("current_rc_override", "legacy_rc_override", "current_rc_out")
CONFIG_LABELS = {
    "current_rc_override": "current\\nRC override",
    "legacy_rc_override": "legacy\\nRC override",
    "current_rc_out": "current\\nRC out",
}
COLORS = {
    "current_rc_override": "#2f6fdd",
    "legacy_rc_override": "#c55338",
    "current_rc_out": "#2d9b66",
}


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def safe_get(mapping: dict[str, Any], path: list[str], default: Any = math.nan) -> Any:
    cur: Any = mapping
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def metric(summary: dict[str, Any], bag: str, cfg: str, group: str, axis: str | None, key: str) -> float:
    replay = summary["bags"][bag]["replays"][cfg]
    if axis is None:
        value = replay.get(group, {}).get(key, math.nan)
    else:
        value = replay.get(group, {}).get(axis, {}).get(key, math.nan)
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def gain(summary: dict[str, Any], bag: str, cfg: str, group: str, axis: str | None) -> float:
    replay = summary["bags"][bag]["replays"][cfg]
    if axis is None:
        fit = replay.get(group, {}).get("gain_fit_real_from_sim", {})
    else:
        fit = replay.get(group, {}).get(axis, {}).get("gain_fit_real_from_sim", {})
    try:
        return float(fit.get("gain", math.nan))
    except (TypeError, ValueError):
        return math.nan


def bag_short(name: str) -> str:
    return name.replace("bag_2026-04-01_", "")


def setup_fig(width: float = 12.0, height: float = 6.0):
    fig = plt.figure(figsize=(width, height))
    return fig


def save(fig, name: str) -> str:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    path = FIG_DIR / name
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return f"real_bag_2026_04_01_replay/latex_figures/{name}"


def plot_pipeline() -> str:
    fig = setup_fig(13.5, 4.0)
    ax = fig.add_subplot(111)
    ax.axis("off")
    boxes = [
        ("April 1\\nreal rosbag", 0.05, 0.58, "#dce8ff"),
        ("RC override\\nRC out", 0.22, 0.58, "#e8f4e4"),
        ("command\\nnormalization", 0.39, 0.58, "#fff1cc"),
        ("MuJoCo replay\\ncurrent / legacy", 0.56, 0.58, "#ffe4dc"),
        ("sensor truth\\nDVL IMU depth", 0.73, 0.58, "#e7e1ff"),
        ("metrics\\nRMSE corr gain lag", 0.88, 0.58, "#eeeeee"),
    ]
    for text, x, y, color in boxes:
        ax.text(
            x,
            y,
            text,
            ha="center",
            va="center",
            fontsize=12,
            bbox=dict(boxstyle="round,pad=0.45", facecolor=color, edgecolor="#555555", linewidth=1.2),
            transform=ax.transAxes,
        )
    for i in range(len(boxes) - 1):
        ax.annotate(
            "",
            xy=(boxes[i + 1][1] - 0.075, boxes[i + 1][2]),
            xytext=(boxes[i][1] + 0.075, boxes[i][2]),
            arrowprops=dict(arrowstyle="->", lw=1.8, color="#333333"),
            xycoords=ax.transAxes,
        )
    ax.text(
        0.5,
        0.20,
        "No ArduPilot package modification. Replay and diagnostics run from document/docsource + uuv_mujoco only.",
        ha="center",
        va="center",
        fontsize=11,
        color="#444444",
        transform=ax.transAxes,
    )
    return save(fig, "april1_replay_pipeline.png")


def plot_sensor_rates(summary: dict[str, Any]) -> str:
    labels = [bag_short(b) for b in BAGS]
    topics = [
        ("IMU", "imu_gyro", "norm"),
        ("DVL", "dvl", "norm"),
        ("Depth", "depth", None),
        ("Static pressure", "static_pressure", None),
    ]
    values = np.full((len(topics), len(BAGS)), np.nan)
    for j, bag in enumerate(BAGS):
        stats = summary["bags"][bag]["real_sensor_stats"]
        for i, (_, key, axis) in enumerate(topics):
            block = stats.get(key, {})
            if axis is not None:
                block = block.get(axis, {})
            rate = block.get("median_rate_hz")
            if rate is not None:
                values[i, j] = float(rate)
    fig = setup_fig(11.0, 5.0)
    ax = fig.add_subplot(111)
    x = np.arange(len(BAGS))
    width = 0.18
    for i, (label, _, _) in enumerate(topics):
        ax.bar(x + (i - 1.5) * width, values[i], width=width, label=label)
    ax.axhline(20, color="#777777", lw=1, ls="--", alpha=0.4)
    ax.axhline(10, color="#777777", lw=1, ls="--", alpha=0.4)
    ax.axhline(2, color="#777777", lw=1, ls="--", alpha=0.4)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("median publish rate [Hz]")
    ax.set_title("Real bag sensor rates")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(ncol=4, loc="upper right")
    return save(fig, "april1_sensor_rates.png")


def plot_command_mapping() -> str:
    cmd = load_json(CMD_DIR / "command_response_summary.json")
    responses = [
        ("dvl_x_m_s", "DVL x", 5),
        ("dvl_y_m_s", "DVL y", 6),
        ("imu_yaw_rate_rad_s", "gyro z", 4),
        ("depth_rate_m_s", "depth rate", 3),
    ]
    result_by_bag = {item["bag"]: item for item in cmd["results"]}
    fig = setup_fig(12.0, 5.2)
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    x = np.arange(len(responses))
    width = 0.36
    for bi, bag in enumerate(MAIN_BAGS):
        result = result_by_bag[bag]
        corrs = []
        lags = []
        for response, _, expected_ch in responses:
            singles = result["responses"].get(response, {}).get("best_single_channels", [])
            best = next((item for item in singles if int(item.get("channel", -1)) == expected_ch), singles[0] if singles else {})
            corrs.append(float(best.get("corr", np.nan)))
            lags.append(float(best.get("lag_s", np.nan)))
        offset = (bi - 0.5) * width
        ax1.bar(x + offset, corrs, width=width, label=bag_short(bag))
        ax2.bar(x + offset, lags, width=width, label=bag_short(bag))
    ax1.axhline(0, color="#333333", lw=0.8)
    ax1.set_xticks(x)
    ax1.set_xticklabels([label for _, label, _ in responses], rotation=20)
    ax1.set_ylabel("signed correlation")
    ax1.set_title("RC override -> measured response")
    ax1.grid(axis="y", alpha=0.25)
    ax1.legend()
    ax2.set_xticks(x)
    ax2.set_xticklabels([label for _, label, _ in responses], rotation=20)
    ax2.set_ylabel("best lag [s]")
    ax2.set_title("response lag")
    ax2.grid(axis="y", alpha=0.25)
    return save(fig, "april1_command_mapping.png")


def plot_replay_rmse(summary: dict[str, Any]) -> str:
    panels = [
        ("DVL x RMSE [m/s]", "dvl_velocity_metrics", "x", False),
        ("DVL y RMSE [m/s]", "dvl_velocity_metrics", "y", False),
        ("gyro z RMSE [rad/s]", "imu_gyro_metrics", "z", False),
        ("depth RMSE [m] (log)", "depth_metrics", None, True),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(13.0, 8.0), sharex=False)
    axes = axes.reshape(-1)
    x = np.arange(len(MAIN_BAGS))
    width = 0.24
    for ax, (title, group, axis, logy) in zip(axes, panels):
        for ci, cfg in enumerate(CONFIGS):
            vals = [metric(summary, bag, cfg, group, axis, "rmse") for bag in MAIN_BAGS]
            ax.bar(x + (ci - 1) * width, vals, width=width, label=cfg.replace("_", " "), color=COLORS[cfg])
        ax.set_xticks(x)
        ax.set_xticklabels([bag_short(b) for b in MAIN_BAGS])
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25)
        if logy:
            ax.set_yscale("log")
    axes[0].legend(ncol=3, fontsize=8)
    return save(fig, "april1_replay_rmse.png")


def plot_corr_gain(summary: dict[str, Any]) -> str:
    metrics = [
        ("DVL x", "dvl_velocity_metrics", "x"),
        ("DVL y", "dvl_velocity_metrics", "y"),
        ("gyro z", "imu_gyro_metrics", "z"),
    ]
    fig, axes = plt.subplots(2, len(MAIN_BAGS), figsize=(13.0, 6.6), sharey="row")
    for col, bag in enumerate(MAIN_BAGS):
        corr_matrix = np.array([[metric(summary, bag, cfg, group, axis, "correlation") for _, group, axis in metrics] for cfg in CONFIGS])
        gain_matrix = np.array([[gain(summary, bag, cfg, group, axis) for _, group, axis in metrics] for cfg in CONFIGS])
        im = axes[0, col].imshow(corr_matrix, vmin=-1, vmax=1, cmap="coolwarm")
        axes[0, col].set_title(f"{bag_short(bag)} correlation")
        axes[0, col].set_xticks(np.arange(len(metrics)))
        axes[0, col].set_xticklabels([m[0] for m in metrics])
        axes[0, col].set_yticks(np.arange(len(CONFIGS)))
        axes[0, col].set_yticklabels([c.replace("_", "\n") for c in CONFIGS])
        for i in range(len(CONFIGS)):
            for j in range(len(metrics)):
                axes[0, col].text(j, i, f"{corr_matrix[i, j]:.2f}", ha="center", va="center", fontsize=9)
        im2 = axes[1, col].imshow(gain_matrix, vmin=-0.3, vmax=1.0, cmap="viridis")
        axes[1, col].set_title(f"{bag_short(bag)} real/sim gain")
        axes[1, col].set_xticks(np.arange(len(metrics)))
        axes[1, col].set_xticklabels([m[0] for m in metrics])
        axes[1, col].set_yticks(np.arange(len(CONFIGS)))
        axes[1, col].set_yticklabels([c.replace("_", "\n") for c in CONFIGS])
        for i in range(len(CONFIGS)):
            for j in range(len(metrics)):
                axes[1, col].text(j, i, f"{gain_matrix[i, j]:.2f}", ha="center", va="center", fontsize=9, color="white" if gain_matrix[i, j] < 0.55 else "black")
    fig.colorbar(im, ax=axes[0, :], shrink=0.82, label="signed correlation")
    fig.colorbar(im2, ax=axes[1, :], shrink=0.82, label="gain")
    return save(fig, "april1_corr_gain_heatmap.png")


def plot_depth_contact() -> str:
    fig, axes = plt.subplots(len(MAIN_BAGS), 1, figsize=(13.0, 6.0), sharex=False)
    for ax, bag in zip(axes, MAIN_BAGS):
        data = np.load(REPLAY_DIR / bag / "current_rc_override_timeseries.npz")
        t = data["t"]
        depth = data["depth"]
        contact = data["contact_count"] > 0
        ax.plot(t, depth, color="#2f6fdd", lw=0.9, label="sim depth")
        if np.any(contact):
            ax.fill_between(t, 0, np.nanmax(depth), where=contact, color="#d55e00", alpha=0.18, label="contact active")
            ax.axvline(t[np.argmax(contact)], color="#d55e00", lw=1.2, ls="--")
        ax.set_title(f"{bag_short(bag)} current RC override depth/contact")
        ax.set_ylabel("depth [m]")
        ax.grid(alpha=0.25)
        ax.legend(loc="upper left")
    axes[-1].set_xlabel("bag time [s]")
    return save(fig, "april1_depth_contact.png")


def plot_sim_truth_sensor() -> str:
    fig, axes = plt.subplots(3, len(MAIN_BAGS), figsize=(14.0, 8.0), sharex="col")
    for col, bag in enumerate(MAIN_BAGS):
        data = np.load(REPLAY_DIR / bag / "current_rc_override_timeseries.npz")
        t = data["t"]
        vel_body = data["vel_body"]
        dvl_vel = data["dvl_vel"]
        dvl_raw = data["dvl_vel_raw_sensor"]
        ang_body = data["ang_body"]
        imu_gyro = data["imu_gyro"]
        base_xyz = data["base_xyz"]
        depth_sensor = data["depth"]
        base_depth_truth = np.maximum(0.0, -base_xyz[:, 2])

        ax = axes[0, col]
        ax.plot(t, vel_body[:, 0], color="#1b4f9c", lw=0.9, label="truth body u")
        ax.plot(t, dvl_vel[:, 0], color="#e07a5f", lw=0.7, alpha=0.85, label="DVL x sensor")
        ax.plot(t, vel_body[:, 1], color="#1b9c7c", lw=0.8, ls="--", label="truth body v")
        ax.plot(t, dvl_vel[:, 1], color="#d39c21", lw=0.7, ls="--", alpha=0.85, label="DVL y sensor")
        ax.plot(t, dvl_raw[:, 1], color="#666666", lw=0.55, alpha=0.35, label="raw DVL y")
        ax.set_title(f"{bag_short(bag)} DVL sensor vs MuJoCo truth")
        ax.set_ylabel("velocity [m/s]")
        ax.grid(alpha=0.25)
        if col == 0:
            ax.legend(fontsize=8, ncol=2)

        ax = axes[1, col]
        ax.plot(t, ang_body[:, 2], color="#1b4f9c", lw=0.9, label="truth body r")
        ax.plot(t, imu_gyro[:, 2], color="#e07a5f", lw=0.7, alpha=0.9, label="IMU gyro z")
        ax.set_title(f"{bag_short(bag)} IMU gyro vs MuJoCo truth")
        ax.set_ylabel("yaw rate [rad/s]")
        ax.grid(alpha=0.25)
        if col == 0:
            ax.legend(fontsize=8)

        ax = axes[2, col]
        ax.plot(t, base_depth_truth, color="#1b4f9c", lw=0.9, label="truth base depth")
        ax.plot(t, depth_sensor, color="#e07a5f", lw=0.8, alpha=0.9, label="Bar30/depth sensor")
        ax.set_title(f"{bag_short(bag)} depth sensor vs MuJoCo truth")
        ax.set_xlabel("bag time [s]")
        ax.set_ylabel("depth [m]")
        ax.grid(alpha=0.25)
        if col == 0:
            ax.legend(fontsize=8)
    return save(fig, "april1_sim_truth_sensor.png")


def plot_frame_validation(summary: dict[str, Any]) -> str:
    labels = []
    corrected_corr = []
    raw_corr = []
    corrected_gain = []
    raw_gain = []
    for bag in MAIN_BAGS:
        block = summary["bags"][bag]["replays"]["current_rc_override"]["sensor_truth_consistency"]
        corrected = block["dvl_published_body_vs_body_truth"]["y"]
        raw = block["dvl_raw_sensor_vs_body_truth"]["y"]
        labels.append(bag_short(bag))
        corrected_corr.append(float(corrected.get("correlation", np.nan)))
        raw_corr.append(float(raw.get("correlation", np.nan)))
        corrected_gain.append(float(corrected.get("gain_fit_measured_from_truth", {}).get("gain", np.nan)))
        raw_gain.append(float(raw.get("gain_fit_measured_from_truth", {}).get("gain", np.nan)))

    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.8), sharex=True)
    x = np.arange(len(labels))
    width = 0.36
    axes[0].bar(x - width / 2, corrected_corr, width=width, label="bridge-corrected")
    axes[0].bar(x + width / 2, raw_corr, width=width, label="raw site-frame")
    axes[0].axhline(0, color="#333333", lw=0.8)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels)
    axes[0].set_ylabel("corr with body truth")
    axes[0].set_title("DVL y frame-validation correlation")
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend()
    axes[1].bar(x - width / 2, corrected_gain, width=width, label="bridge-corrected")
    axes[1].bar(x + width / 2, raw_gain, width=width, label="raw site-frame")
    axes[1].axhline(1.0, color="#333333", lw=0.8, ls="--", alpha=0.7)
    axes[1].axhline(0.0, color="#333333", lw=0.8)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels)
    axes[1].set_ylabel("measured/truth gain")
    axes[1].set_title("DVL y frame-validation gain")
    axes[1].grid(axis="y", alpha=0.25)
    return save(fig, "april1_dvl_frame_validation.png")


def plot_thruster_p95(summary: dict[str, Any]) -> str:
    names = ("ver_lf", "ver_lr", "ver_rf", "ver_rr", "yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr")
    fig, axes = plt.subplots(1, len(MAIN_BAGS), figsize=(14.0, 4.8), sharey=True)
    for ax, bag in zip(axes, MAIN_BAGS):
        replay = summary["bags"][bag]["replays"]["current_rc_override"]
        vals = [safe_get(replay, ["thruster_summary", name, "force_n", "p95"], math.nan) for name in names]
        ax.bar(np.arange(len(names)), vals, color=["#6aaed6"] * 4 + ["#e07a5f"] * 4)
        ax.set_xticks(np.arange(len(names)))
        ax.set_xticklabels(names, rotation=45, ha="right")
        ax.set_title(f"{bag_short(bag)} p95 thruster force")
        ax.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel("force p95 [N]")
    return save(fig, "april1_thruster_p95.png")


def plot_pressure_semantics(summary: dict[str, Any]) -> str:
    labels = [bag_short(b) for b in BAGS]
    static_range = []
    atm_range = []
    depth_range = []
    for bag in BAGS:
        stats = summary["bags"][bag]["real_sensor_stats"]
        static_range.append(float(stats.get("static_pressure", {}).get("range", np.nan)))
        atm_range.append(float(stats.get("atm_pressure", {}).get("range", np.nan)))
        depth_range.append(float(stats.get("depth", {}).get("range", np.nan)))
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.8))
    x = np.arange(len(BAGS))
    width = 0.36
    axes[0].bar(x - width / 2, static_range, width=width, label="static_pressure")
    axes[0].bar(x + width / 2, atm_range, width=width, label="atm_pressure")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels, rotation=10)
    axes[0].set_ylabel("pressure range [Pa]")
    axes[0].set_title("Real pressure-topic semantics")
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend()
    axes[1].bar(x, depth_range, color="#4c956c")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, rotation=10)
    axes[1].set_ylabel("depth range [m]")
    axes[1].set_title("Depth range from /depth/pose")
    axes[1].grid(axis="y", alpha=0.25)
    return save(fig, "april1_pressure_semantics.png")


def write_tex(figs: dict[str, str]) -> None:
    tex = textwrap.dedent(
        r"""
        \documentclass[11pt,a4paper]{{article}}

        \usepackage{{fontspec}}
        \usepackage[margin=18mm]{{geometry}}
        \usepackage{{graphicx}}
        \usepackage{{booktabs}}
        \usepackage{{array}}
        \usepackage{{tabularx}}
        \usepackage{{caption}}
        \usepackage{{subcaption}}
        \usepackage{{float}}
        \usepackage{{hyperref}}
        \usepackage{{xcolor}}
        \usepackage{{enumitem}}

        \defaultfontfeatures{{Ligatures=TeX,Scale=MatchLowercase}}
        \IfFontExistsTF{{Noto Sans CJK KR}}{{
          \setmainfont{{Noto Sans CJK KR}}[
            ItalicFeatures={{FakeSlant=0.18}},
            BoldItalicFeatures={{FakeSlant=0.18}}
          ]
          \setsansfont{{Noto Sans CJK KR}}[
            ItalicFeatures={{FakeSlant=0.18}},
            BoldItalicFeatures={{FakeSlant=0.18}}
          ]
        }}{{
          \IfFontExistsTF{{Apple SD Gothic Neo}}{{
            \setmainfont{{Apple SD Gothic Neo}}[
              ItalicFeatures={{FakeSlant=0.18}},
              BoldItalicFeatures={{FakeSlant=0.18}}
            ]
            \setsansfont{{Apple SD Gothic Neo}}[
              ItalicFeatures={{FakeSlant=0.18}},
              BoldItalicFeatures={{FakeSlant=0.18}}
            ]
          }}{{
            \setmainfont{{NanumGothic}}[
              ItalicFeatures={{FakeSlant=0.18}},
              BoldItalicFeatures={{FakeSlant=0.18}}
            ]
            \setsansfont{{NanumGothic}}[
              ItalicFeatures={{FakeSlant=0.18}},
              BoldItalicFeatures={{FakeSlant=0.18}}
            ]
          }}
        }}
        \IfFontExistsTF{{DejaVu Sans Mono}}{{\setmonofont{{DejaVu Sans Mono}}}}{{\setmonofont{{Menlo}}}}
        \XeTeXlinebreaklocale "ko"
        \XeTeXlinebreakskip = 0pt plus 1pt

        \hypersetup{{
          colorlinks=true,
          linkcolor=blue!50!black,
          urlcolor=blue!50!black,
          pdftitle={{2026-04-01 UUV MuJoCo replay analysis}},
          pdfauthor={{OpenAI Codex}}
        }}
        \setlength{{\parskip}}{{0.45em}}
        \setlength{{\parindent}}{{0pt}}
        \renewcommand{{\arraystretch}}{{1.18}}
        \newcolumntype{{Y}}{{>{{\raggedright\arraybackslash}}X}}
        \newcommand{{\code}}[1]{{\texttt{{#1}}}}
        \captionsetup{{font=small,labelfont=bf,justification=raggedright,singlelinecheck=false}}

        \title{{2026-04-01 실제 로봇 rosbag 기반 UUV MuJoCo Replay 분석\\
        \large 그래프 중심 진단 보고서}}
        \author{{}}
        \date{{2026-04-28}}

        \begin{{document}}
        \maketitle

        \section{{분석 파이프라인}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{@@PIPELINE@@}
          \caption{{4월 1일 실제 rosbag을 읽어 RC 입력을 정규화하고, current/legacy MuJoCo 모델에 replay한 뒤 DVL, IMU, depth, pressure, odometry metric을 계산한 구조. ArduPilot package는 수정하지 않았다.}}
        \end{{figure}}

        \section{{실제 bag 센서 구성}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.94\linewidth]{@@SENSOR_RATES@@}
          \caption{{실제 bag의 주요 센서 rate. IMU는 약 20 Hz, DVL은 약 10 Hz, depth/static pressure는 약 2 Hz로 확인된다. 20:06 bag에는 DVL twist가 없다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.94\linewidth]{@@PRESSURE@@}
          \caption{{실제 pressure topic 의미. \code{/mavros/imu/static\_pressure}는 depth 변화와 함께 변하고, \code{/mavros/imu/atm\_pressure}는 0.24 근처 상수다. 이 결과에 맞춰 bridge 기본값을 수정했다.}}
        \end{{figure}}

        \section{{RC override 명령-응답 구조}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.94\linewidth]{@@COMMAND_MAPPING@@}
          \caption{{실제 bag에서 RC override와 센서 응답의 상관/지연. ch5는 DVL x, ch6는 DVL y, ch4는 yaw rate, ch3는 depth rate 후보로 해석된다. 단, ch3/heave는 두 bag 모두 식별력이 약하다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \begin{{subfigure}}{{0.49\linewidth}}
            \includegraphics[width=\linewidth]{{real_bag_2026_04_01_command_response/bag_2026-04-01_20-08-11_command_response.png}}
            \caption{{20:08 command-response overlay}}
          \end{{subfigure}}
          \begin{{subfigure}}{{0.49\linewidth}}
            \includegraphics[width=\linewidth]{{real_bag_2026_04_01_command_response/bag_2026-04-01_20-20-30_command_response.png}}
            \caption{{20:20 command-response overlay}}
          \end{{subfigure}}
          \caption{{실제 RC override와 DVL/gyro/depth 응답의 시간축 비교. surge와 yaw는 비교 가능한 수준으로 명령-응답 구조가 보이고, heave는 분리된 step이 부족하다.}}
        \end{{figure}}

        \section{{MuJoCo replay 정확도}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{@@RMSE@@}
          \caption{{real-vs-sim replay RMSE. current RC override는 DVL x와 yaw rate의 형태는 일부 맞지만 속도/각속도 scale이 크다. legacy RC override는 현재 실제 bag 기준으로 훨씬 크게 발산한다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{@@CORR_GAIN@@}
          \caption{{상관계수와 real/sim gain heatmap. DVL x, DVL y, gyro z 모두 current RC override에서 일부 형태가 맞지만 gain이 1보다 훨씬 작다. 이는 sim response amplitude가 실제보다 크다는 뜻이다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \begin{{subfigure}}{{0.49\linewidth}}
            \includegraphics[width=\linewidth]{{real_bag_2026_04_01_replay/bag_2026-04-01_20-08-11/current_rc_override_overlay.png}}
            \caption{{20:08 current RC override replay}}
          \end{{subfigure}}
          \begin{{subfigure}}{{0.49\linewidth}}
            \includegraphics[width=\linewidth]{{real_bag_2026_04_01_replay/bag_2026-04-01_20-20-30/current_rc_override_overlay.png}}
            \caption{{20:20 current RC override replay}}
          \end{{subfigure}}
          \caption{{실제 센서와 current MuJoCo replay overlay. DVL x/y와 yaw rate는 일부 형태가 맞지만 depth와 trajectory는 controller 생략 및 contact 이후 발산이 크다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{@@SIM_TRUTH_SENSOR@@}
          \caption{{같은 실제 RC override 명령을 current MuJoCo 모델에 넣었을 때의 MuJoCo 내부 truth와 bridge-corrected MuJoCo 센서 출력 비교. raw DVL site-frame y는 body-frame truth와 부호가 반대지만, bridge 보정 후 실제 \code{/dvl/twist} 비교값은 body truth와 일치한다. 실제 rosbag에는 외부 motion-capture/ground-truth pose가 없으므로 real truth 비교는 불가능하다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.92\linewidth]{@@FRAME_VALIDATION@@}
          \caption{{DVL y frame-validation. raw MuJoCo DVL sensor는 \code{dvl\_site}의 180도 회전 때문에 body y와 음의 상관을 갖지만, bridge-corrected DVL은 body truth와 corr $\simeq 1$, gain $\simeq 1$로 일치한다. 따라서 이전 negative DVL y 판단은 물리 문제가 아니라 replay 계측 오류였다.}}
        \end{{figure}}

        \section{{발산 원인 그래프}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{@@DEPTH_CONTACT@@}
          \caption{{current RC override replay의 depth와 contact timeline. 20:08은 약 5.72 s, 20:20은 약 15.18 s에 contact가 발생한다. contact 이후의 trajectory/depth metric은 물리 model fit보다는 발산 결과로 봐야 한다.}}
        \end{{figure}}

        \begin{{figure}}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{@@THRUSTER@@}
          \caption{{current RC override replay에서 thruster force p95. T200 성능곡선 위에 \code{gain\_scale\_all=2.9}가 다시 곱해져 yaw thruster force가 비현실적으로 커진다.}}
        \end{{figure}}

        \section{{정량 결론}}

        \begin{{table}}[H]
          \centering
          \caption{{핵심 해석 요약}}
          \begin{{tabularx}}{{\linewidth}}{{lY}}
            \toprule
            항목 & 판단 \\
            \midrule
            DVL x / surge & current RC override에서 20:08 corr 0.738, 20:20 corr 0.356. 형태는 일부 맞지만 real/sim gain이 0.254, 0.082라 sim 속도가 과하다. \\
            DVL y / sway & bridge-corrected 비교 후 20:08 corr 0.505, 20:20 corr 0.504로 양의 상관이 확인된다. 단 real/sim gain이 0.150, 0.103이라 sim sway amplitude가 과하다. \\
            IMU yaw rate & 20:08 corr 0.872로 형태는 좋지만 real/sim gain 0.351. sim yaw rate가 실제보다 약 2.9배 크다. \\
            Depth / pressure & 실제 depth pressure는 static pressure topic에 있다. bridge pressure semantics는 수정 완료. depth 동역학은 ALT\_HOLD controller 생략 때문에 direct replay로 평가하면 안 된다. \\
            물리 파라미터 & \code{gain\_scale\_all=2.9}와 \code{body\_inertia\_scale\_xyz=[1e-6,...]}가 가장 먼저 손봐야 할 값이다. \\
            \bottomrule
          \end{{tabularx}}
        \end{{table}}

        \section{{수정 우선순위}}

        \begin{{enumerate}}[leftmargin=2em]
          \item T200 curve 사용 시 \code{gain\_scale\_all}을 1.0 근처로 되돌리고 bag 기반 thruster gain을 재식별한다.
          \item current profile의 \code{body\_inertia\_scale\_xyz=[1e-6,...]}를 제거하고 CAD/질량분포 기반 inertia를 사용한다.
          \item DVL/IMU/depth frame-validation은 regression test로 고정하고, 이후 replay 비교는 bridge-corrected sensor output만 사용한다.
          \item heave/depth는 RC override direct replay가 아니라 SITL/ALT\_HOLD controller-in-loop replay로 다시 비교한다.
          \item 그 다음에 damping, added mass, cross-flow drag, DVL dropout/bottom-lock, camera/sonar realism을 추가한다.
        \end{{enumerate}}

        \end{{document}}
        """
    ).strip()
    tex = tex.replace("{{", "{").replace("}}", "}")
    replacements = {
        "@@PIPELINE@@": figs["pipeline"],
        "@@SENSOR_RATES@@": figs["sensor_rates"],
        "@@PRESSURE@@": figs["pressure"],
        "@@COMMAND_MAPPING@@": figs["command_mapping"],
        "@@RMSE@@": figs["rmse"],
        "@@CORR_GAIN@@": figs["corr_gain"],
        "@@SIM_TRUTH_SENSOR@@": figs["sim_truth_sensor"],
        "@@FRAME_VALIDATION@@": figs["frame_validation"],
        "@@DEPTH_CONTACT@@": figs["depth_contact"],
        "@@THRUSTER@@": figs["thruster"],
    }
    for key, value in replacements.items():
        tex = tex.replace(key, value)
    TEX_PATH.write_text(tex)


def main() -> None:
    summary = load_json(REPLAY_DIR / "summary.json")
    figs = {
        "pipeline": plot_pipeline(),
        "sensor_rates": plot_sensor_rates(summary),
        "pressure": plot_pressure_semantics(summary),
        "command_mapping": plot_command_mapping(),
        "rmse": plot_replay_rmse(summary),
        "corr_gain": plot_corr_gain(summary),
        "sim_truth_sensor": plot_sim_truth_sensor(),
        "frame_validation": plot_frame_validation(summary),
        "depth_contact": plot_depth_contact(),
        "thruster": plot_thruster_p95(summary),
    }
    write_tex(figs)
    print(TEX_PATH)
    for path in figs.values():
        print(DOCSRC / path)


if __name__ == "__main__":
    main()

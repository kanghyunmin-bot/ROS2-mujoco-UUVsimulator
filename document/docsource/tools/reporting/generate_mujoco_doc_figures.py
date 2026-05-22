from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DOCSRC_DIR = Path(__file__).resolve().parent
FIG_DIR = DOCSRC_DIR / "figures"
SIM_DIR = ROOT / "uuv_mujoco" / "v2.2"

sys.path.insert(0, str(SIM_DIR))

from physics.sim_profile_helpers import build_hydrodynamics_config, build_sim_profile, load_sim_profiles


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Apple SD Gothic Neo", "Helvetica", "Arial", "DejaVu Sans"],
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "figure.titlesize": 12,
        "axes.unicode_minus": False,
    }
)


def ensure_dir() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)


def load_thruster_performance() -> dict:
    return json.loads((SIM_DIR / "config" / "thruster_performance.json").read_text())


def load_thruster_params() -> dict:
    return json.loads((SIM_DIR / "config" / "thruster_params.json").read_text())


def load_sim_real_cfg():
    profiles, warning = load_sim_profiles(SIM_DIR / "config" / "sim_profiles.json")
    if warning is not None:
        raise RuntimeError(warning)
    sim_profile = build_sim_profile(profiles, "sim_real")
    return build_hydrodynamics_config(sim_profile, fluid_density=1000.0), sim_profile


def plot_thruster_curve() -> None:
    obj = load_thruster_performance()
    curves = {curve["voltage_v"]: curve for curve in obj["curves"]}

    fig, ax = plt.subplots(figsize=(7.8, 4.8), dpi=220, constrained_layout=True)
    for voltage, color in [(10.0, "#4c78a8"), (16.0, "#f58518"), (20.0, "#54a24b")]:
        curve = curves[voltage]
        ax.plot(curve["pwm_us"], curve["force_n"], label=f"{int(voltage)} V", linewidth=2.0, color=color)

    curve16 = curves[16.0]
    pwm = np.asarray(curve16["pwm_us"], dtype=float)
    force = np.asarray(curve16["force_n"], dtype=float)
    neutral_pwm = float(obj["meta"]["neutral_pwm"])
    fmax = float(np.max(force))
    fmin = float(np.min(force))
    linear_force = np.where(
        pwm >= neutral_pwm,
        ((pwm - neutral_pwm) / (1900.0 - neutral_pwm)) * fmax,
        ((pwm - neutral_pwm) / (neutral_pwm - 1100.0)) * abs(fmin),
    )
    ax.plot(pwm, linear_force, linestyle="--", color="#333333", linewidth=1.5, label="16 V linear fallback")
    ax.axvline(neutral_pwm, color="#888888", linewidth=1.0, linestyle=":")
    ax.axhline(0.0, color="#aaaaaa", linewidth=0.8)
    ax.set_title("T200 thrust curve: measured PWM-force relationship")
    ax.set_xlabel("PWM [us]")
    ax.set_ylabel("Force [N]")
    ax.grid(True, alpha=0.25)
    ax.legend(frameon=False, ncol=2, loc="lower right")
    fig.savefig(FIG_DIR / "thruster_curve_comparison.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_hydrodynamic_coeffs() -> None:
    cfg, _ = load_sim_real_cfg()
    labels_t = ["surge", "sway", "heave"]
    labels_r = ["roll", "pitch", "yaw"]
    added = cfg.added_mass_diag
    linear = cfg.linear_damping_diag
    quadratic = cfg.quadratic_damping_diag

    fig, axes = plt.subplots(1, 2, figsize=(10.8, 5.3), dpi=220, constrained_layout=True)
    width = 0.24

    x = np.arange(3)
    axes[0].bar(x - width, added[:3], width, label="Added mass", color="#4c78a8")
    axes[0].bar(x, linear[:3], width, label="Linear damping", color="#f58518")
    axes[0].bar(x + width, quadratic[:3], width, label="Quadratic damping", color="#54a24b")
    axes[0].set_xticks(x, labels_t)
    axes[0].set_title("Translational axes (sim_real)")
    axes[0].set_ylabel("Coefficient value")
    axes[0].grid(True, axis="y", alpha=0.25)

    axes[1].bar(x - width, added[3:], width, label="Added inertia", color="#4c78a8")
    axes[1].bar(x, linear[3:], width, label="Linear damping", color="#f58518")
    axes[1].bar(x + width, quadratic[3:], width, label="Quadratic damping", color="#54a24b")
    axes[1].set_xticks(x, labels_r)
    axes[1].set_title("Rotational axes (sim_real)")
    axes[1].grid(True, axis="y", alpha=0.25)

    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=3,
        frameon=False,
        bbox_to_anchor=(0.5, 0.98),
        columnspacing=1.4,
        handlelength=1.8,
    )
    fig.suptitle("Current hydrodynamic coefficients\nfrom ellipsoid baseline", y=1.05)
    fig.savefig(FIG_DIR / "sim_real_hydrodynamic_coeffs.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_buoyancy_curve() -> None:
    cfg, _ = load_sim_real_cfg()
    rho = 1000.0
    g = 9.81
    vehicle_mass = 10.463
    half_height = float(cfg.half_height)
    neutral_volume = vehicle_mass / rho

    depth = np.linspace(-0.25, 0.25, 500)
    submerged = np.clip((depth + half_height) / (2.0 * half_height), 0.0, 1.0)
    buoy = rho * g * neutral_volume * submerged * float(cfg.buoyancy_scale)

    fig, ax1 = plt.subplots(figsize=(7.6, 4.6), dpi=220, constrained_layout=True)
    ax2 = ax1.twinx()
    ax1.plot(depth, submerged, color="#4c78a8", linewidth=2.2, label="Submerged ratio")
    ax2.plot(depth, buoy, color="#f58518", linewidth=2.2, label="Buoyancy force")
    ax1.axvline(-half_height, color="#999999", linestyle=":", linewidth=1.0)
    ax1.axvline(half_height, color="#999999", linestyle=":", linewidth=1.0)
    ax1.axvline(0.0, color="#bbbbbb", linestyle="--", linewidth=0.8)
    ax1.set_title("Submergence ratio and buoyancy in current model")
    ax1.set_xlabel("Depth of base origin relative to water surface [m]")
    ax1.set_ylabel("Submerged ratio")
    ax2.set_ylabel("Buoyancy force [N]")
    ax1.set_ylim(-0.05, 1.05)
    ax1.grid(True, alpha=0.25)
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left", frameon=False)
    fig.savefig(FIG_DIR / "buoyancy_submergence_curve.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_thruster_gain_calibration() -> None:
    params = load_thruster_params()
    items = list(params["per_thruster"].items())
    names = [k for k, _ in items]
    gains = np.array([v["gain_scale"] for _, v in items], dtype=float)

    fig, ax = plt.subplots(figsize=(9.4, 5.0), dpi=220, constrained_layout=True)
    colors = ["#4c78a8" if n.startswith("ver_") else "#54a24b" for n in names]
    ax.bar(np.arange(len(names)), gains, color=colors, width=0.72)
    ax.axhline(1.0, color="#333333", linestyle="--", linewidth=1.0)
    ax.set_xticks(np.arange(len(names)), names, rotation=32, ha="right")
    ax.set_ylabel("Gain scale")
    ax.set_title("Per-thruster gain calibration values")
    ax.set_ylim(0.90, 1.08)
    ax.grid(True, axis="y", alpha=0.25)
    fig.savefig(FIG_DIR / "thruster_gain_calibration.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def main() -> None:
    ensure_dir()
    plot_thruster_curve()
    plot_hydrodynamic_coeffs()
    plot_buoyancy_curve()
    plot_thruster_gain_calibration()
    print(f"generated figures in {FIG_DIR}")


if __name__ == "__main__":
    main()

"""Panel renderers for ALT_HOLD contract plots."""

from __future__ import annotations

import numpy as np

from althold_contract_model import RC_NEUTRAL


def _combined_legend(ax, other_ax) -> None:
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = other_ax.get_legend_handles_labels()
    ax.legend(lines + lines2, labels + labels2, loc="best", fontsize=8)


def plot_pilot_heave(ax, t: np.ndarray, rc3: np.ndarray) -> None:
    ax.step(t, rc3, where="post", color="tab:gray", label="RCIN.C3")
    ax.axhline(RC_NEUTRAL, color="black", linewidth=0.8, alpha=0.5)
    ax.set_title("Pilot Heave Input")
    ax.set_ylabel("PWM")
    ax.legend(loc="best", fontsize=8)


def plot_vertical_target(
    ax,
    t: np.ndarray,
    expected: np.ndarray,
    dcrt: np.ndarray,
    pscd_tvd: np.ndarray,
    pscd_vd: np.ndarray,
) -> None:
    ax.plot(t, expected, color="tab:green", label="expected climb from RC3")
    ax.plot(t, dcrt, color="tab:red", label="CTUN.DCRt actual target")
    ax.plot(t, pscd_tvd, color="tab:purple", alpha=0.75, label="PSCD.TVD")
    ax.plot(t, pscd_vd, color="tab:brown", alpha=0.75, label="PSCD.VD")
    ax.axhline(0, color="black", linewidth=0.8, alpha=0.5)
    ax.set_title("ALT_HOLD Vertical Target Contract")
    ax.set_ylabel("cm/s")
    ax.legend(loc="best", fontsize=8)


def plot_depth_velocity(
    ax,
    t: np.ndarray,
    sim_pd: np.ndarray,
    sim_vd: np.ndarray,
    visv_vz: np.ndarray,
) -> None:
    ax.plot(t, sim_pd, color="tab:red", label="SIM2.PD depth")
    depth_ax = ax.twinx()
    depth_ax.plot(t, sim_vd, color="tab:orange", alpha=0.8, label="SIM2.VD")
    depth_ax.plot(t, visv_vz, color="tab:blue", alpha=0.7, label="VISV.VZ")
    ax.set_title("Plant Depth and EKF ExternalNav Velocity")
    ax.set_ylabel("m down")
    depth_ax.set_ylabel("m/s down")
    _combined_legend(ax, depth_ax)


def plot_vertical_outputs(
    ax,
    t: np.ndarray,
    rcou: dict[str, np.ndarray],
    plant_cmd: np.ndarray,
) -> None:
    for key, color in zip(["C5", "C6", "C7", "C8"], ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd"]):
        ax.step(t, rcou[key], where="post", label=f"RCOU.{key}", color=color)
    ax.axhline(RC_NEUTRAL, color="black", linewidth=0.8, alpha=0.5)
    cmd_ax = ax.twinx()
    cmd_ax.step(t, plant_cmd, where="post", color="tab:red", label="plant vertical cmd (down +)", linewidth=1.5)
    ax.set_title("Vertical Motor Outputs")
    ax.set_ylabel("PWM")
    cmd_ax.set_ylabel("norm")
    _combined_legend(ax, cmd_ax)


def plot_attitude_coupling(
    ax,
    t: np.ndarray,
    att: dict[str, np.ndarray],
    rate: dict[str, np.ndarray],
) -> None:
    ax.plot(t, att["Roll"], label="Roll", color="tab:blue")
    ax.plot(t, att["Pitch"], label="Pitch", color="tab:orange")
    rate_ax = ax.twinx()
    rate_ax.plot(t, rate["ROut"], label="RATE.ROut", color="tab:green", alpha=0.7)
    rate_ax.plot(t, rate["POut"], label="RATE.POut", color="tab:red", alpha=0.7)
    ax.set_title("Attitude Coupling")
    ax.set_ylabel("deg")
    rate_ax.set_ylabel("controller output")
    ax.set_xlabel("time [s]")
    _combined_legend(ax, rate_ax)


__all__ = [
    "plot_attitude_coupling",
    "plot_depth_velocity",
    "plot_pilot_heave",
    "plot_vertical_outputs",
    "plot_vertical_target",
]

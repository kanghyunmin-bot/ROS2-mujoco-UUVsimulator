"""Plotting helpers for ALT_HOLD contract analysis."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from althold_contract_plot_modes import prepare_axes, shade_modes
from althold_contract_plot_panels import (
    plot_attitude_coupling,
    plot_depth_velocity,
    plot_pilot_heave,
    plot_vertical_outputs,
    plot_vertical_target,
)


def write_plot(
    path: Path,
    t: np.ndarray,
    modes: list[dict],
    rc3: np.ndarray,
    expected: np.ndarray,
    dcrt: np.ndarray,
    pscd_tvd: np.ndarray,
    pscd_vd: np.ndarray,
    sim_pd: np.ndarray,
    sim_vd: np.ndarray,
    visv_vz: np.ndarray,
    rcou: dict[str, np.ndarray],
    plant_cmd: np.ndarray,
    att: dict[str, np.ndarray],
    rate: dict[str, np.ndarray],
) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(5, 1, figsize=(14, 13), sharex=True)
    end_t = float(t[-1]) if len(t) else 0.0
    prepare_axes(axes, modes, end_t)

    plot_pilot_heave(axes[0], t, rc3)
    plot_vertical_target(axes[1], t, expected, dcrt, pscd_tvd, pscd_vd)
    plot_depth_velocity(axes[2], t, sim_pd, sim_vd, visv_vz)
    plot_vertical_outputs(axes[3], t, rcou, plant_cmd)
    plot_attitude_coupling(axes[4], t, att, rate)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


__all__ = ["shade_modes", "write_plot"]

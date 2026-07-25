"""PNG rendering for axis RC response plots."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from axis_rc_contract import Phase
from axis_rc_plot_series import dvl_speed_series, relative_time, values


def plot_timeseries(path: Path, samples: list[dict[str, Any]], phases: list[Phase]) -> None:
    if not samples:
        return
    t = relative_time(samples)
    dvl_vx, dvl_vy, dvl_vz, dvl_speed = dvl_speed_series(samples)
    fig, axes = plt.subplots(4, 1, figsize=(12, 8.5), sharex=True)

    _plot_depth_panel(axes[0], t, samples)
    _plot_dvl_panel(axes[1], t, dvl_vx, dvl_vy, dvl_vz, dvl_speed)
    _plot_gyro_panel(axes[2], t, samples)
    _plot_rc_panel(axes[3], t, samples)
    _decorate_axes(axes, samples, phases)

    axes[-1].set_xlabel("time s")
    fig.suptitle(path.parent.name)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _plot_depth_panel(axis: Any, t: list[float], samples: list[dict[str, Any]]) -> None:
    axis.plot(t, values(samples, "depth_m"), label="Bar30 depth", lw=1.1)
    axis.plot(t, values(samples, "odom_z"), label="local z", lw=0.9, alpha=0.65)
    axis.set_ylabel("depth m")


def _plot_dvl_panel(
    axis: Any,
    t: list[float],
    dvl_vx: list[float],
    dvl_vy: list[float],
    dvl_vz: list[float],
    dvl_speed: list[float],
) -> None:
    axis.plot(t, dvl_vx, label="vx", lw=0.9)
    axis.plot(t, dvl_vy, label="vy", lw=0.9)
    axis.plot(t, dvl_vz, label="vz", lw=0.9)
    axis.plot(t, dvl_speed, label="speed", lw=1.1, alpha=0.8)
    axis.set_ylabel("DVL m/s")


def _plot_gyro_panel(axis: Any, t: list[float], samples: list[dict[str, Any]]) -> None:
    axis.plot(t, values(samples, "gyro_x"), label="gyro x", lw=0.9)
    axis.plot(t, values(samples, "gyro_y"), label="gyro y", lw=0.9)
    axis.plot(t, values(samples, "gyro_z"), label="gyro z", lw=0.9)
    axis.set_ylabel("gyro rad/s")


def _plot_rc_panel(axis: Any, t: list[float], samples: list[dict[str, Any]]) -> None:
    for idx in range(1, 9):
        rcin = values(samples, f"rcin{idx}")
        if any(math.isfinite(v) for v in rcin):
            axis.step(t, rcin, where="post", label=f"ch{idx}", lw=0.8)
    axis.set_ylabel("RC pwm")


def _decorate_axes(axes: Any, samples: list[dict[str, Any]], phases: list[Phase]) -> None:
    t0 = float(samples[0]["t"])
    for axis in axes:
        for phase in phases:
            if phase.axis != "neutral":
                axis.axvspan(phase.start - t0, phase.end - t0, color="0.2", alpha=0.035, lw=0)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right", ncol=4, fontsize=8)


__all__ = ["plot_timeseries"]

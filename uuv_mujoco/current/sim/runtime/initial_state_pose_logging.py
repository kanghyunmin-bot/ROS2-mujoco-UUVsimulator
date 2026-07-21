"""Log helpers for initial pose overrides."""

from __future__ import annotations

import numpy as np


def log_initial_horizontal_position(initial_xy: np.ndarray) -> None:
    print(
        "[runtime] initial horizontal position set: "
        f"x={initial_xy[0]:+.4f} y={initial_xy[1]:+.4f} m",
        flush=True,
    )


def log_initial_attitude(initial_rpy: np.ndarray) -> None:
    print(
        "[runtime] initial attitude set: "
        f"roll={initial_rpy[0]:+.4f} pitch={initial_rpy[1]:+.4f} yaw={initial_rpy[2]:+.4f} rad",
        flush=True,
    )


__all__ = ["log_initial_attitude", "log_initial_horizontal_position"]

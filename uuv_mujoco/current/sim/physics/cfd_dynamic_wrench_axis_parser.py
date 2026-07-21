"""Axis-table parsing for CFD-derived dynamic wrench profiles."""

from __future__ import annotations

from typing import Any, Callable, Optional

import numpy as np

from sim.physics.cfd_dynamic_wrench_axis_tables import (
    log_invalid_cfd_axis,
    nonnegative_float_table,
    valid_cfd_axis_force_tables,
)

AXIS_TO_INDEX = {"x": 0, "y": 1, "z": 2}


def parse_cfd_dynamic_wrench_axes(
    cfg: dict[str, Any],
    *,
    to_float_array: Callable[[Any], Optional[np.ndarray]],
    log: Optional[Callable[[str], None]],
) -> dict[str, dict[str, object]]:
    raw_axes = cfg.get("axes", {})
    if not isinstance(raw_axes, dict):
        raw_axes = {}
    axes: dict[str, dict[str, object]] = {}
    for axis_name, axis_index in AXIS_TO_INDEX.items():
        axis_cfg = raw_axes.get(axis_name, {})
        parsed = _parse_axis(axis_name, axis_index, axis_cfg, to_float_array=to_float_array, log=log)
        if parsed is not None:
            axes[axis_name] = parsed
    return axes


def _parse_axis(
    axis_name: str,
    axis_index: int,
    axis_cfg: Any,
    *,
    to_float_array: Callable[[Any], Optional[np.ndarray]],
    log: Optional[Callable[[str], None]],
) -> dict[str, object] | None:
    if not isinstance(axis_cfg, dict):
        return None
    speeds = to_float_array(axis_cfg.get("speeds_mps"))
    positive = to_float_array(axis_cfg.get("positive_velocity_force_n"))
    negative = to_float_array(axis_cfg.get("negative_velocity_force_n"))
    if not valid_cfd_axis_force_tables(speeds, positive, negative):
        log_invalid_cfd_axis(axis_name, log)
        return None
    return {
        "index": axis_index,
        "speeds": nonnegative_float_table(speeds),
        "positive": nonnegative_float_table(positive),
        "negative": nonnegative_float_table(negative),
        "extrapolate": str(axis_cfg.get("extrapolate", "linear")),
    }


__all__ = ["AXIS_TO_INDEX", "parse_cfd_dynamic_wrench_axes"]

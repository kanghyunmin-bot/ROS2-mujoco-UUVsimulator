"""Telemetry sample construction for axis RC checks."""

from __future__ import annotations

from typing import Any

from axis_rc_sample_channels import add_channels
from axis_rc_sample_motion import add_depth, add_dvl, add_imu, add_odom


def build_axis_sample(
    *,
    elapsed_s: float,
    wall_mono_s: float,
    phase: str,
    state: Any,
    imu: Any,
    dvl_twist: Any,
    depth: Any,
    local_odom: Any,
    rc_in: Any,
    rc_out: Any,
    command_axis: str | None = None,
    command_value: float = 0.0,
    command_publish_t: float = float("nan"),
    command_sequence: int = 0,
    command_mode: str = "",
) -> dict[str, Any]:
    sample: dict[str, Any] = {
        "t": float(elapsed_s),
        "wall_mono_s": float(wall_mono_s),
        "phase": phase,
        "command_axis": str(command_axis or ""),
        "command_value": float(command_value),
        "command_publish_t": float(command_publish_t),
        "command_age_s": (
            max(0.0, float(elapsed_s) - float(command_publish_t))
            if command_publish_t == command_publish_t
            else float("nan")
        ),
        "command_sequence": int(command_sequence),
        "command_mode": str(command_mode),
        "mode": str(state.mode) if state is not None else "",
        "armed": bool(state.armed) if state is not None else False,
        "manual_input": bool(getattr(state, "manual_input", False)) if state is not None else False,
    }
    add_imu(sample, imu)
    add_dvl(sample, dvl_twist)
    add_odom(sample, local_odom)
    add_depth(sample, depth)
    add_channels(sample, "rcin", rc_in)
    add_channels(sample, "rcout", rc_out)
    return sample


__all__ = ["build_axis_sample"]

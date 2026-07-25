"""Runtime thruster geometry overrides."""

from __future__ import annotations

import os
from typing import Any, Callable

from sim.physics.actuator_geometry import (
    apply_horizontal_thruster_z_offset,
    apply_vertical_thruster_x_scale,
)


def vertical_scale_center_x() -> float | None:
    raw = os.environ.get("UUV_VERTICAL_THRUSTER_X_SCALE_CENTER_M", "").strip()
    try:
        return float(raw) if raw else None
    except ValueError:
        return None


def apply_thruster_geometry_overrides(
    *,
    mujoco_module: Any,
    model: Any,
    sim_profile: dict[str, Any],
    actuator_ids: dict[str, int],
    vertical_thrusters: list[str],
    horizontal_thrusters: list[str],
    env_float: Callable[[str, float], float],
    log: Callable[[str], None],
) -> None:
    apply_horizontal_thruster_z_offset(
        model=model,
        mujoco_module=mujoco_module,
        actuator_ids=actuator_ids,
        thruster_names=horizontal_thrusters,
        offset_m=env_float(
            "UUV_HORIZONTAL_THRUSTER_Z_OFFSET_M",
            float(sim_profile.get("horizontal_thruster_z_offset_m", 0.0)),
        ),
        log=log,
    )
    apply_vertical_thruster_x_scale(
        model=model,
        mujoco_module=mujoco_module,
        actuator_ids=actuator_ids,
        thruster_names=vertical_thrusters,
        scale_raw=env_float(
            "UUV_VERTICAL_THRUSTER_X_SCALE",
            float(sim_profile.get("vertical_thruster_x_scale", 1.0)),
        ),
        center_x=vertical_scale_center_x(),
        log=log,
    )


__all__ = ["apply_thruster_geometry_overrides", "vertical_scale_center_x"]

"""Horizontal thruster site geometry adjustments."""

from __future__ import annotations

from typing import Any, Callable, Mapping, Sequence

import numpy as np

from sim.physics.actuator_geometry_sites import actuator_site_id


def apply_horizontal_thruster_z_offset(
    *,
    model: Any,
    mujoco_module: Any,
    actuator_ids: Mapping[str, int],
    thruster_names: Sequence[str],
    offset_m: float,
    log: Callable[[str], None],
) -> None:
    if abs(float(offset_m)) <= 1.0e-9:
        return
    adjusted = []
    clipped_offset = float(np.clip(float(offset_m), -0.2, 0.2))
    for name in thruster_names:
        site_id = actuator_site_id(
            model=model,
            mujoco_module=mujoco_module,
            actuator_ids=actuator_ids,
            name=name,
        )
        if site_id < 0:
            continue
        old_z = float(model.site_pos[site_id, 2])
        model.site_pos[site_id, 2] = old_z + clipped_offset
        adjusted.append(f"{name}:{old_z:.4f}->{float(model.site_pos[site_id, 2]):.4f}")
    if adjusted:
        log(
            "[thruster] horizontal force application z offset: "
            f"UUV_HORIZONTAL_THRUSTER_Z_OFFSET_M={float(offset_m):.4f} "
            + ", ".join(adjusted)
        )


__all__ = ["apply_horizontal_thruster_z_offset"]

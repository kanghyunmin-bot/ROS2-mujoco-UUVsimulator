"""Vertical thruster lever-arm geometry adjustments."""

from __future__ import annotations

from typing import Any, Callable, Mapping, Sequence

from sim.physics.actuator_geometry_vertical_scale import (
    apply_vertical_x_scale_to_sites,
    clipped_vertical_x_scale,
    vertical_scale_center_x,
    vertical_thruster_site_ids,
)


def apply_vertical_thruster_x_scale(
    *,
    model: Any,
    mujoco_module: Any,
    actuator_ids: Mapping[str, int],
    thruster_names: Sequence[str],
    scale_raw: float,
    center_x: float | None,
    log: Callable[[str], None],
) -> None:
    if abs(float(scale_raw) - 1.0) <= 1.0e-9:
        return
    site_ids = vertical_thruster_site_ids(
        model=model,
        mujoco_module=mujoco_module,
        actuator_ids=actuator_ids,
        thruster_names=thruster_names,
    )
    if not site_ids:
        return
    center = vertical_scale_center_x(model, site_ids=site_ids, center_x=center_x)
    scale = clipped_vertical_x_scale(scale_raw)
    adjusted = apply_vertical_x_scale_to_sites(
        model=model,
        mujoco_module=mujoco_module,
        actuator_ids=actuator_ids,
        thruster_names=thruster_names,
        center=center,
        scale=scale,
    )
    if adjusted:
        log(
            "[thruster] vertical pitch lever x scale: "
            f"UUV_VERTICAL_THRUSTER_X_SCALE={scale:.4f} center={center:+.4f} "
            + ", ".join(adjusted)
        )


__all__ = ["apply_vertical_thruster_x_scale"]

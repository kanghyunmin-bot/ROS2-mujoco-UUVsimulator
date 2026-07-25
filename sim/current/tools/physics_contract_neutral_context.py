"""Context construction for neutral open-plant buoyancy checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class NeutralBuoyancyContext:
    base_id: int
    world_qpos_adr: int
    water_surface_z: float
    rho: float
    gravity: float
    neutral_volume: float
    buoyancy_scale: float
    hydro_cfg: Any
    components: tuple[Any, ...]
    total_share: float
    cob_longitudinal_offset: float
    cob_torque_scale: float


def build_neutral_buoyancy_context(
    *,
    base_id: int,
    world_qpos_adr: int,
    water_surface_z: float,
    rho: float,
    gravity: float,
    neutral_volume: float,
    buoyancy_scale: float,
    hydro_cfg: Any,
    sim_profile: dict[str, Any],
) -> NeutralBuoyancyContext:
    components = tuple(hydro_cfg.body_components)
    total_share = float(sum(c.buoyancy_share for c in components))
    if total_share <= 1.0e-9:
        total_share = float(sum(c.mass for c in components))
    if total_share <= 1.0e-9:
        raise RuntimeError("neutral open-plant simulation requires body_components")
    return NeutralBuoyancyContext(
        base_id=base_id,
        world_qpos_adr=world_qpos_adr,
        water_surface_z=float(water_surface_z),
        rho=float(rho),
        gravity=float(gravity),
        neutral_volume=float(neutral_volume),
        buoyancy_scale=float(buoyancy_scale),
        hydro_cfg=hydro_cfg,
        components=components,
        total_share=total_share,
        cob_longitudinal_offset=float(sim_profile.get("cob_x_offset", 0.0)),
        cob_torque_scale=float(hydro_cfg.cob_torque_scale),
    )


__all__ = ["NeutralBuoyancyContext", "build_neutral_buoyancy_context"]

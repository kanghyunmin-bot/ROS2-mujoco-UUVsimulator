"""Vehicle mass and neutral-volume reference helpers."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from sim.physics.model_setup import body_subtree_mass


@dataclass(frozen=True)
class MassReference:
    rho: float
    gravity: float
    vehicle_mass: float
    neutral_volume: float


def build_mass_reference(
    *,
    np_module: Any,
    model: Any,
    base_id: int,
    scene_fluid_density: float,
    log: Callable[[str], None],
    subtree_mass_fn: Callable[[Any, int], float] = body_subtree_mass,
) -> MassReference:
    rho = float(scene_fluid_density)
    gravity = abs(float(model.opt.gravity[2]))
    total_mass_all = float(np_module.sum(model.body_mass[1:]))
    vehicle_mass = float(subtree_mass_fn(model, base_id))
    if vehicle_mass <= 1e-9:
        vehicle_mass = float(model.body_mass[base_id])
    neutral_volume = vehicle_mass / max(rho, 1e-6)
    if total_mass_all > vehicle_mass * 1.2:
        log(
            "[physics] buoyancy mass reference: "
            f"vehicle_subtree={vehicle_mass:.3f}kg (all_nonworld={total_mass_all:.3f}kg)"
        )
    return MassReference(
        rho=rho,
        gravity=gravity,
        vehicle_mass=vehicle_mass,
        neutral_volume=neutral_volume,
    )


__all__ = ["MassReference", "build_mass_reference"]

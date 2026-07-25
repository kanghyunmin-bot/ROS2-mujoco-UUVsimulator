"""Hydrostatic buoyancy calculations for physics contract audits."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from physics.hydrodynamics_helpers import submerged_fraction  # noqa: E402
from physics_contract_types import ForceBalance  # noqa: E402


def component_half_height(component: Any) -> float:
    try:
        return float(max(component.size[2], 1.0e-4))
    except Exception:
        return 1.0e-4


def component_share(component: Any, total_share: float) -> float:
    return float(component.buoyancy_share / max(total_share, 1.0e-9))


def weighted_submerged_fraction(base_depth_m: float, hydro_cfg: Any) -> float:
    total_share = float(sum(c.buoyancy_share for c in hydro_cfg.body_components))
    if total_share <= 1.0e-9:
        total_share = float(sum(c.mass for c in hydro_cfg.body_components))
    if total_share <= 1.0e-9:
        return 0.0

    weighted = 0.0
    for component in hydro_cfg.body_components:
        share = component_share(component, total_share)
        depth = float(base_depth_m - float(component.buoyancy_pos[2]))
        half_height = component_half_height(component)
        weighted += share * submerged_fraction(
            depth * float(hydro_cfg.buoyancy_slope_scale),
            half_height,
            str(hydro_cfg.buoyancy_model),
        )
    return float(np.clip(weighted, 0.0, 1.0))


def force_balance(
    *,
    label: str,
    base_depth_m: float,
    bar30_local_z: float,
    vehicle_mass: float,
    rho: float,
    gravity: float,
    neutral_volume: float,
    buoyancy_scale: float,
    hydro_cfg: Any,
) -> ForceBalance:
    submerged = weighted_submerged_fraction(base_depth_m, hydro_cfg)
    buoyancy_n = float(rho * gravity * neutral_volume * buoyancy_scale * submerged)
    weight_n = float(vehicle_mass * gravity)
    net_up_n = float(buoyancy_n - weight_n)
    net_down_n = float(weight_n - buoyancy_n)
    accel_down = float(net_down_n / max(vehicle_mass, 1.0e-9))
    denom = float(rho * gravity * neutral_volume * max(submerged, 1.0e-9))
    required_scale = float(weight_n / denom)
    return ForceBalance(
        label=label,
        base_depth_m=float(base_depth_m),
        bar30_depth_m=float(base_depth_m - bar30_local_z),
        buoyancy_n=buoyancy_n,
        weight_n=weight_n,
        net_up_n=net_up_n,
        net_down_n=net_down_n,
        accel_down_mps2=accel_down,
        submerged_fraction=submerged,
        required_buoyancy_scale=required_scale,
    )

"""Compatibility facade for MuJoCo static physics contract calculations."""

from __future__ import annotations

import numpy as np

from physics_contract_body import apply_runtime_body_contract, component_self_inertia_diag
from physics_contract_buoyancy import (
    component_half_height,
    component_share,
    force_balance,
    weighted_submerged_fraction,
)
from physics_contract_geometry import (
    body_subtree_mass,
    fluid_geom_ids,
    geom_local_top_z,
    rpy_rad_from_quat_wxyz,
    set_base_depth,
    site_local_z,
)
from physics_contract_mujoco import mujoco
from physics_contract_neutral_sim import simulate_neutral_open_plant

__all__ = [
    "apply_runtime_body_contract",
    "body_subtree_mass",
    "component_half_height",
    "component_self_inertia_diag",
    "component_share",
    "fluid_geom_ids",
    "force_balance",
    "geom_local_top_z",
    "mujoco",
    "np",
    "rpy_rad_from_quat_wxyz",
    "set_base_depth",
    "simulate_neutral_open_plant",
    "site_local_z",
    "weighted_submerged_fraction",
]

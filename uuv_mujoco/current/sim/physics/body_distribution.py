"""Compatibility exports for body component mass/CoM/inertia distribution."""

from __future__ import annotations

from sim.physics.body_distribution_inertia import component_self_inertia_diag
from sim.physics.body_distribution_runtime import apply_body_component_distribution


__all__ = ["apply_body_component_distribution", "component_self_inertia_diag"]

"""Depth candidate assembly for static physics contract audits."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from physics_contract_model import site_local_z
from physics_contract_runner_context import PhysicsContractAuditContext
from physics_contract_start_depth import auto_depths_from_candidates, build_start_depth_candidates


@dataclass
class PhysicsContractAuditDepths:
    bar30_local_z: float
    surface_depth_m: float
    start_candidates: list[tuple[str, float]]
    auto_bar30_depth: float
    auto_base_depth: float
    audited_depths: list[tuple[str, float]]


def build_physics_contract_audit_depths(
    *,
    args: Any,
    context: PhysicsContractAuditContext,
) -> PhysicsContractAuditDepths:
    bar30_local_z = site_local_z(context.model, "bar30_site") or 0.0
    surface_depth_m = abs(float(args.surface_depth_cm)) / 100.0
    start_candidates = build_start_depth_candidates(
        model=context.model,
        base_id=context.base_id,
        bar30_local_z=bar30_local_z,
        hydro_cfg=context.hydro_cfg,
        surface_depth_m=surface_depth_m,
        minimum_bar30_depth_m=float(args.minimum_bar30_depth_m),
        thruster_half_height_m=float(args.thruster_half_height_m),
        margin_m=float(args.margin_m),
    )
    auto_bar30_depth, auto_base_depth = auto_depths_from_candidates(start_candidates, bar30_local_z)

    audited_depths: list[tuple[str, float]] = [
        ("scene_default_base_depth", -float(context.model.body_pos[context.base_id, 2])),
        ("auto_fully_wet_base_depth", auto_base_depth),
    ]
    for depth in args.depth:
        audited_depths.append((f"requested_base_depth_{depth:.3f}", float(depth)))

    return PhysicsContractAuditDepths(
        bar30_local_z=bar30_local_z,
        surface_depth_m=surface_depth_m,
        start_candidates=start_candidates,
        auto_bar30_depth=auto_bar30_depth,
        auto_base_depth=auto_base_depth,
        audited_depths=audited_depths,
    )


__all__ = ["PhysicsContractAuditDepths", "build_physics_contract_audit_depths"]

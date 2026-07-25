"""Force balance calculation for static physics contract audits."""

from __future__ import annotations

from physics_contract_model import force_balance
from physics_contract_runner_context import PhysicsContractAuditContext
from physics_contract_runner_depths import PhysicsContractAuditDepths
from physics_contract_types import ForceBalance


def build_physics_contract_force_balances(
    *,
    context: PhysicsContractAuditContext,
    depths: PhysicsContractAuditDepths,
) -> list[ForceBalance]:
    return [
        force_balance(
            label=label,
            base_depth_m=depth,
            bar30_local_z=depths.bar30_local_z,
            vehicle_mass=context.vehicle_mass,
            rho=context.rho,
            gravity=context.gravity,
            neutral_volume=context.neutral_volume,
            buoyancy_scale=context.buoyancy_scale,
            hydro_cfg=context.hydro_cfg,
        )
        for label, depth in depths.audited_depths
    ]


__all__ = ["build_physics_contract_force_balances"]

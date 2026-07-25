"""Report assembly and output for static physics contract audits."""

from __future__ import annotations

from physics_contract_audit_report import build_physics_contract_report
from physics_contract_report import print_physics_contract_report, write_static_force_balance_outputs
from physics_contract_runner_context import PhysicsContractAuditContext
from physics_contract_runner_depths import PhysicsContractAuditDepths
from physics_contract_types import ForceBalance, NeutralSimSummary


def write_and_print_physics_contract_audit(
    *,
    context: PhysicsContractAuditContext,
    depths: PhysicsContractAuditDepths,
    balances: list[ForceBalance],
    neutral_sims: list[NeutralSimSummary],
    profile_name: str,
) -> None:
    report = build_physics_contract_report(
        scene=context.scene,
        profile_path=context.profile_path,
        profile_name=profile_name,
        vehicle_mass=context.vehicle_mass,
        gravity=context.gravity,
        rho=context.rho,
        neutral_volume=context.neutral_volume,
        buoyancy_scale=context.buoyancy_scale,
        body_contract=context.body_contract,
        sim_profile=context.sim_profile,
        hydro_cfg=context.hydro_cfg,
        bar30_local_z=depths.bar30_local_z,
        surface_depth_m=depths.surface_depth_m,
        auto_bar30_depth=depths.auto_bar30_depth,
        auto_base_depth=depths.auto_base_depth,
        start_candidates=depths.start_candidates,
        balances=balances,
        neutral_sims=neutral_sims,
    )
    csv_path, json_path = write_static_force_balance_outputs(
        output_dir=context.output_dir,
        report=report,
        balances=balances,
    )
    print_physics_contract_report(
        body_contract=context.body_contract,
        vehicle_mass=context.vehicle_mass,
        gravity=context.gravity,
        rho=context.rho,
        neutral_volume=context.neutral_volume,
        buoyancy_scale=context.buoyancy_scale,
        hydro_cfg=context.hydro_cfg,
        auto_bar30_depth=depths.auto_bar30_depth,
        auto_base_depth=depths.auto_base_depth,
        start_candidates=depths.start_candidates,
        balances=balances,
        neutral_sims=neutral_sims,
        csv_path=csv_path,
        json_path=json_path,
    )


__all__ = ["write_and_print_physics_contract_audit"]

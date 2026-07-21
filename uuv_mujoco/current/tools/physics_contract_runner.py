"""Runner for static MuJoCo physics contract audits."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from physics_contract_neutral_runner import run_neutral_open_plant_sims  # noqa: E402
from physics_contract_runner_balances import build_physics_contract_force_balances  # noqa: E402
from physics_contract_runner_context import build_physics_contract_audit_context  # noqa: E402
from physics_contract_runner_depths import build_physics_contract_audit_depths  # noqa: E402
from physics_contract_runner_outputs import write_and_print_physics_contract_audit  # noqa: E402


def run_physics_contract_audit(args: Any) -> int:
    context = build_physics_contract_audit_context(args)
    depths = build_physics_contract_audit_depths(args=args, context=context)
    balances = build_physics_contract_force_balances(context=context, depths=depths)
    neutral_sims = run_neutral_open_plant_sims(
        args=args,
        model=context.model,
        base_id=context.base_id,
        audited_depths=depths.audited_depths,
        output_dir=context.output_dir,
        vehicle_mass=context.vehicle_mass,
        rho=context.rho,
        gravity=context.gravity,
        neutral_volume=context.neutral_volume,
        buoyancy_scale=context.buoyancy_scale,
        hydro_cfg=context.hydro_cfg,
        sim_profile=context.sim_profile,
    )
    write_and_print_physics_contract_audit(
        context=context,
        depths=depths,
        balances=balances,
        neutral_sims=neutral_sims,
        profile_name=args.profile,
    )
    return 0


__all__ = ["run_physics_contract_audit"]

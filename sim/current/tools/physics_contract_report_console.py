"""Console report orchestration for static physics contract audits."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from physics_contract_report_sections import (
    print_balance_section,
    print_hydrostatic_section,
    print_mass_section,
    print_neutral_sims_section,
    print_output_paths,
    print_start_depth_section,
)
from physics_contract_types import BodyContract, ForceBalance, NeutralSimSummary


def print_physics_contract_report(
    *,
    body_contract: BodyContract,
    vehicle_mass: float,
    gravity: float,
    rho: float,
    neutral_volume: float,
    buoyancy_scale: float,
    hydro_cfg: Any,
    auto_bar30_depth: float,
    auto_base_depth: float,
    start_candidates: list[tuple[str, float]],
    balances: list[ForceBalance],
    neutral_sims: list[NeutralSimSummary],
    csv_path: Path,
    json_path: Path,
) -> None:
    print("[physics-audit] static force balance")
    print_mass_section(body_contract, vehicle_mass=vehicle_mass, gravity=gravity)
    print_hydrostatic_section(
        rho=rho,
        neutral_volume=neutral_volume,
        buoyancy_scale=buoyancy_scale,
        hydro_cfg=hydro_cfg,
    )
    print_start_depth_section(
        auto_bar30_depth=auto_bar30_depth,
        auto_base_depth=auto_base_depth,
        start_candidates=start_candidates,
    )
    print_balance_section(balances)
    print_neutral_sims_section(neutral_sims)
    print_output_paths(csv_path, json_path)


__all__ = ["print_physics_contract_report"]

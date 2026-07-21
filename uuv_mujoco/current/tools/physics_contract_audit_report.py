"""Report dictionary assembly for static physics contract audits."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from physics_contract_types import BodyContract, ForceBalance, NeutralSimSummary


def build_physics_contract_report(
    *,
    scene: Path,
    profile_path: Path,
    profile_name: str,
    vehicle_mass: float,
    gravity: float,
    rho: float,
    neutral_volume: float,
    buoyancy_scale: float,
    body_contract: BodyContract,
    sim_profile: dict[str, Any],
    hydro_cfg: Any,
    bar30_local_z: float,
    surface_depth_m: float,
    auto_bar30_depth: float,
    auto_base_depth: float,
    start_candidates: list[tuple[str, float]],
    balances: list[ForceBalance],
    neutral_sims: list[NeutralSimSummary],
) -> dict[str, Any]:
    return {
        "scene": str(scene),
        "profile_file": str(profile_path),
        "profile": profile_name,
        "vehicle_mass_kg": vehicle_mass,
        "gravity_mps2": gravity,
        "rho_kgm3": rho,
        "neutral_volume_m3": neutral_volume,
        "buoyancy_scale": buoyancy_scale,
        "body_contract": body_contract.__dict__,
        "cob_x_offset": float(sim_profile.get("cob_x_offset", 0.0)),
        "cob_z_offset": float(sim_profile.get("cob_z_offset", 0.0)),
        "cob_torque_scale": float(hydro_cfg.cob_torque_scale),
        "hydrostatic_volume_source": str(hydro_cfg.hydrostatic_volume_source),
        "hydrostatic_restoring_active": bool(hydro_cfg.hydrostatic_restoring_active),
        "hydrostatic_restoring_roll_stiffness_nm_per_rad": float(hydro_cfg.hydrostatic_restoring_roll_stiffness),
        "hydrostatic_restoring_pitch_stiffness_nm_per_rad": float(hydro_cfg.hydrostatic_restoring_pitch_stiffness),
        "bar30_local_z_m": bar30_local_z,
        "surface_depth_m": surface_depth_m,
        "auto_bar30_depth_m": auto_bar30_depth,
        "auto_base_depth_m": auto_base_depth,
        "start_depth_candidates_m": dict(sorted(start_candidates, key=lambda item: item[1], reverse=True)),
        "balances": [row.__dict__ for row in balances],
        "neutral_open_plant": [row.__dict__ for row in neutral_sims],
    }


__all__ = ["build_physics_contract_report"]

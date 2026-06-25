"""Simulation profile loading and audit-only overrides for physics contract audits."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from physics.sim_profile_helpers import build_hydrodynamics_config, build_sim_profile, load_sim_profiles


def resolve_physics_contract_profile(
    profile_path: Path,
    profile_name: str,
    args: Any,
    rho: float,
) -> tuple[dict[str, Any], Any]:
    profiles, warning = load_sim_profiles(profile_path)
    if warning:
        print(warning, flush=True)
    sim_profile = build_sim_profile(profiles, profile_name)
    if args.cob_x_offset is not None:
        sim_profile["cob_x_offset"] = float(args.cob_x_offset)
    if args.cob_z_offset is not None:
        sim_profile["cob_z_offset"] = float(args.cob_z_offset)
    if args.cob_torque_scale is not None:
        sim_profile["cob_torque_scale"] = float(args.cob_torque_scale)
    return sim_profile, build_hydrodynamics_config(sim_profile, fluid_density=rho)


__all__ = ["resolve_physics_contract_profile"]

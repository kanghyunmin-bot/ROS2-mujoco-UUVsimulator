"""Runtime simulation profile selection helpers."""

from __future__ import annotations

from pathlib import Path

from physics.sim_profile_helpers import load_sim_profiles

from sim.physics.profile_runtime_listing import (
    listed_runtime_profile_selection,
    print_available_runtime_profiles,
)
from sim.physics.profile_runtime_selection import (
    build_runtime_sim_profile,
    resolve_runtime_profile_name,
)
from sim.physics.profile_runtime_types import RuntimeProfileSelection
from sim.physics.profile_runtime_voltage import select_profile_thruster_voltage


def select_runtime_profile(args) -> RuntimeProfileSelection:
    """Load profile JSON, resolve profile alias, and choose thruster voltage."""

    profile_path = Path(args.profile_file).expanduser()
    profiles, profile_warning = load_sim_profiles(profile_path)
    if profile_warning is not None:
        print(profile_warning, flush=True)

    if args.list_profiles:
        print_available_runtime_profiles(profiles)
        return listed_runtime_profile_selection(
            profile_path=profile_path,
            profiles=profiles,
        )

    resolved_profile_name = resolve_runtime_profile_name(args.profile, profiles)
    sim_profile = build_runtime_sim_profile(
        profiles=profiles,
        raw_profile_name=args.profile,
        resolved_profile_name=resolved_profile_name,
        profile_path=profile_path,
        buoyancy_scale_override=args.buoyancy_scale,
    )
    active_thruster_voltage = select_profile_thruster_voltage(
        sim_profile=sim_profile,
        override_voltage=args.thruster_voltage,
    )

    return RuntimeProfileSelection(
        profile_path=profile_path,
        profiles=profiles,
        resolved_profile_name=resolved_profile_name,
        sim_profile=sim_profile,
        active_thruster_voltage=active_thruster_voltage,
    )


__all__ = [
    "RuntimeProfileSelection",
    "build_runtime_sim_profile",
    "listed_runtime_profile_selection",
    "print_available_runtime_profiles",
    "resolve_runtime_profile_name",
    "select_profile_thruster_voltage",
    "select_runtime_profile",
]

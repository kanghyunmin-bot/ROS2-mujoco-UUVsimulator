"""Runtime profile body resolution."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from physics.sim_profile_helpers import build_sim_profile, canonical_profile_name

from sim.physics.profile_runtime_listing import print_available_runtime_profiles


def resolve_runtime_profile_name(raw_profile_name: str, profiles: dict[str, Any]) -> str:
    resolved_profile_name = canonical_profile_name(raw_profile_name)
    if resolved_profile_name in profiles:
        return resolved_profile_name

    print(f"[profile] unknown profile: {raw_profile_name}", flush=True)
    print_available_runtime_profiles(profiles)
    raise SystemExit(2)


def build_runtime_sim_profile(
    *,
    profiles: dict[str, Any],
    raw_profile_name: str,
    resolved_profile_name: str,
    profile_path: Path,
    buoyancy_scale_override: float | None,
) -> dict[str, Any]:
    sim_profile = build_sim_profile(
        profiles,
        resolved_profile_name,
        buoyancy_scale_override=buoyancy_scale_override,
    )
    if resolved_profile_name != raw_profile_name:
        print(f"[profile] alias '{raw_profile_name}' -> '{resolved_profile_name}'", flush=True)
    print(f"[profile] using '{resolved_profile_name}' from {profile_path}", flush=True)
    if buoyancy_scale_override is not None:
        print(
            f"[profile] override buoyancy_scale={sim_profile['buoyancy_scale']:.3f}",
            flush=True,
        )
    return sim_profile


__all__ = ["build_runtime_sim_profile", "resolve_runtime_profile_name"]

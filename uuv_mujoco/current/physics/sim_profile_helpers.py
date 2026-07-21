"""Helpers for simulation profile loading and physics knob grouping."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from .sim_profile_defaults import DEFAULT_SIM_PROFILES, PROFILE_ALIASES
from .sim_profile_hydrodynamics import build_hydrodynamics_config
from .sim_profile_types import BodyComponent, BuoyancyPoint, HydrodynamicsConfig
from .sim_profile_validation import validate_sim_profile


def canonical_profile_name(name: str) -> str:
    key = str(name).strip().lower()
    return PROFILE_ALIASES.get(key, key)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def load_sim_profiles(profile_path: Path) -> tuple[dict[str, dict[str, Any]], str | None]:
    """Load profile overrides from JSON and merge with built-in defaults."""

    profile_path = Path(profile_path).expanduser()
    if not profile_path.exists():
        profile_path.write_text(json.dumps(DEFAULT_SIM_PROFILES, indent=2))

    profiles = {name: dict(cfg) for name, cfg in DEFAULT_SIM_PROFILES.items()}
    try:
        payload = json.loads(profile_path.read_text())
    except json.JSONDecodeError:
        return profiles, f"[profile] invalid json: {profile_path}, using built-in defaults"

    if isinstance(payload, dict):
        for name, cfg in payload.items():
            if not isinstance(cfg, dict):
                continue
            canonical_name = canonical_profile_name(name)
            merged = dict(profiles.get(canonical_name, {}))
            merged.update(cfg)
            profiles[canonical_name] = merged
    return profiles, None


def build_sim_profile(
    profiles: Mapping[str, Mapping[str, Any]],
    profile_name: str,
    buoyancy_scale_override: float | None = None,
) -> dict[str, Any]:
    """Return one resolved profile with optional runtime overrides applied."""

    resolved_name = canonical_profile_name(profile_name)
    if resolved_name not in profiles:
        raise KeyError(profile_name)
    sim_profile = dict(profiles[resolved_name])
    if buoyancy_scale_override is not None:
        sim_profile["buoyancy_scale"] = _clamp(float(buoyancy_scale_override), 0.0, 2.0)
    validate_sim_profile(sim_profile, profile_name=resolved_name)
    return sim_profile


__all__ = [
    "PROFILE_ALIASES",
    "DEFAULT_SIM_PROFILES",
    "HydrodynamicsConfig",
    "BodyComponent",
    "BuoyancyPoint",
    "canonical_profile_name",
    "load_sim_profiles",
    "build_sim_profile",
    "build_hydrodynamics_config",
]

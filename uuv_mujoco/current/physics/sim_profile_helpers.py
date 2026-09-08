"""Helpers for simulation profile loading and physics knob grouping."""

from __future__ import annotations

from copy import deepcopy
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

    profiles = deepcopy(DEFAULT_SIM_PROFILES)
    try:
        payload = json.loads(profile_path.read_text())
    except json.JSONDecodeError:
        return profiles, f"[profile] invalid json: {profile_path}, using built-in defaults"

    if isinstance(payload, dict):
        try:
            profiles = _resolve_profile_inheritance(payload, profiles)
        except ValueError as exc:
            return profiles, f"[profile] invalid inheritance: {exc}; using built-in defaults"
    return profiles, None


def _resolve_profile_inheritance(
    payload: Mapping[str, Any],
    defaults: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    """Resolve optional ``extends`` links without mutating parent profiles."""

    raw_profiles = {
        canonical_profile_name(name): deepcopy(cfg)
        for name, cfg in payload.items()
        if isinstance(cfg, dict)
    }
    resolved = deepcopy(defaults)
    resolving: set[str] = set()

    def resolve(name: str) -> dict[str, Any]:
        if name in resolving:
            chain = " -> ".join((*sorted(resolving), name))
            raise ValueError(f"cycle detected ({chain})")
        raw = raw_profiles.get(name)
        if raw is None:
            if name not in resolved:
                raise ValueError(f"unknown parent profile '{name}'")
            return deepcopy(resolved[name])

        resolving.add(name)
        parent_value = raw.get("extends")
        if parent_value is None:
            base = deepcopy(defaults.get(name, {}))
        else:
            if not isinstance(parent_value, str) or not parent_value.strip():
                raise ValueError(f"profile '{name}' extends must be a non-empty string")
            parent = canonical_profile_name(parent_value)
            if parent == name:
                raise ValueError(f"profile '{name}' cannot extend itself")
            base = resolve(parent)
        base.update(deepcopy({key: value for key, value in raw.items() if key != "extends"}))
        resolving.remove(name)
        resolved[name] = base
        return deepcopy(base)

    for profile_name in raw_profiles:
        resolve(profile_name)
    return resolved


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

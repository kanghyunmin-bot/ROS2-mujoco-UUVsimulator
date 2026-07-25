"""Physics profile selection helpers for GUI editing."""

from __future__ import annotations

from typing import Any

from .config import PHYSICS_PROFILE_NAME


def _current_physics_profile(payload: dict[str, Any]) -> dict[str, Any]:
    profiles = payload.get("profiles")
    if isinstance(profiles, dict) and isinstance(profiles.get(PHYSICS_PROFILE_NAME), dict):
        return profiles[PHYSICS_PROFILE_NAME]
    profile = payload.get(PHYSICS_PROFILE_NAME)
    if isinstance(profile, dict):
        return profile
    raise KeyError(f"profile '{PHYSICS_PROFILE_NAME}' not found")


__all__ = ["_current_physics_profile"]

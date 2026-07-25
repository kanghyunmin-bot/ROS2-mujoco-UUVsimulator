"""Profile loading helpers for closed-loop contract audits."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def resolve_active_runtime(workspace: Path) -> tuple[Path, str]:
    active_runtime = workspace / "sim" / "current"
    return active_runtime, "sim/current"


def nested_get(mapping: dict[str, Any], dotted_key: str) -> Any:
    current: Any = mapping
    for part in dotted_key.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def load_profile(path: Path, name: str) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    profiles = payload.get("profiles")
    if isinstance(profiles, dict) and isinstance(profiles.get(name), dict):
        return profiles[name]
    profile = payload.get(name)
    if isinstance(profile, dict):
        return profile
    raise KeyError(f"profile '{name}' not found in {path}")


__all__ = ["load_profile", "nested_get", "resolve_active_runtime"]

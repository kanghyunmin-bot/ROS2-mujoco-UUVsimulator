"""Listing helpers for runtime simulation profiles."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from sim.physics.profile_runtime_types import RuntimeProfileSelection


PROFILE_ALIASES_TEXT = "Aliases: ellipsoid->current, builtin-ellipsoid->current, custom->legacy"


def print_available_runtime_profiles(profiles: dict[str, Any]) -> None:
    print("Available profiles:", flush=True)
    for name in sorted(profiles):
        print(f"  - {name}", flush=True)
    print(PROFILE_ALIASES_TEXT, flush=True)


def listed_runtime_profile_selection(
    *,
    profile_path: Path,
    profiles: dict[str, Any],
) -> RuntimeProfileSelection:
    return RuntimeProfileSelection(
        profile_path=profile_path,
        profiles=profiles,
        resolved_profile_name="",
        sim_profile={},
        active_thruster_voltage=math.nan,
        listed_profiles=True,
    )


__all__ = [
    "PROFILE_ALIASES_TEXT",
    "listed_runtime_profile_selection",
    "print_available_runtime_profiles",
]

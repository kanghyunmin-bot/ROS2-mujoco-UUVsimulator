"""Runtime simulation profile selection types."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RuntimeProfileSelection:
    profile_path: Path
    profiles: dict[str, Any]
    resolved_profile_name: str
    sim_profile: dict[str, Any]
    active_thruster_voltage: float
    listed_profiles: bool = False


__all__ = ["RuntimeProfileSelection"]

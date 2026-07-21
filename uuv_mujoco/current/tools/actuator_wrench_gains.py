"""Direct-gain loading helpers for actuator wrench audits."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from actuator_wrench_common import finite_float, load_json
from physics.thruster_mapping import PHYSICAL_VERTICAL_THRUSTERS, PHYSICAL_YAW_THRUSTERS


def load_direct_gains(path: Path) -> dict[str, float]:
    payload = load_json(path)
    global_scale = finite_float(payload.get("global", {}).get("direct_gain_scale_all"), 1.0)
    gains = {name: 1.0 for name in PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS}
    for name, cfg in payload.get("per_thruster", {}).items():
        if name in gains and isinstance(cfg, dict):
            gains[name] = finite_float(cfg.get("direct_gain_scale"), 1.0) * global_scale
    return gains


def apply_profile_direct_gain_scales(gains: dict[str, float], profile: dict[str, Any]) -> dict[str, float]:
    """Match runtime direct-gain scaling from sim_profiles.json."""
    out = dict(gains)
    scales = profile.get("thruster_direct_gain_scales", {})
    if not isinstance(scales, dict):
        return out
    for name in out:
        out[name] *= finite_float(scales.get(name), 1.0)
    return out


__all__ = ["apply_profile_direct_gain_scales", "load_direct_gains"]

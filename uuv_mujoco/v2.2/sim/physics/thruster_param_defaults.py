"""Default thruster parameter payloads."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable


def default_thruster_global_params() -> dict[str, Any]:
    """Return a fresh copy of the legacy global thruster parameter defaults."""
    return {
        "deadzone": 0.05,
        "tau_up": 0.12,
        "tau_down": 0.18,
        "reverse_asymmetry": 0.75,
        "command_limit": 0.65,
        "reaction_torque_gain": 0.012,
        "gain_scale_all": 1.0,
        "direct_gain_scale_all": 1.0,
        "forward_poly": [0.0, 3.5, 7.0, 12.0],
        "reverse_poly": [0.0, 2.8, 5.5, 9.5],
    }


def default_thruster_params_payload(thruster_names: Iterable[str]) -> dict[str, Any]:
    """Build the on-disk JSON payload used when config/thruster_params.json is missing."""
    names = list(thruster_names)
    return {
        "global": default_thruster_global_params(),
        "per_thruster": {
            name: {
                "gain_scale": 1.0,
                "direct_gain_scale": 1.0,
                "reverse_asymmetry": None,
                "tau_up": None,
                "tau_down": None,
            }
            for name in names
        },
    }


def ensure_thruster_params_file(path: Path, thruster_names: Iterable[str]) -> None:
    """Create the thruster parameter JSON with defaults if it is missing."""
    if path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = default_thruster_params_payload(thruster_names)
    path.write_text(json.dumps(payload, indent=2))


__all__ = [
    "default_thruster_global_params",
    "default_thruster_params_payload",
    "ensure_thruster_params_file",
]

"""State creation helpers for runtime thruster parameters."""

from __future__ import annotations

from typing import Any

from sim.physics.thruster_params import default_thruster_global_params


def create_thruster_parameter_state(thruster_names: list[str]) -> dict[str, Any]:
    return {
        "global_params": default_thruster_global_params(),
        "scale": {name: 1.0 for name in thruster_names},
        "direct_scale": {name: 1.0 for name in thruster_names},
        "reverse_asymmetry": {name: None for name in thruster_names},
        "tau_up": {name: None for name in thruster_names},
        "tau_down": {name: None for name in thruster_names},
    }


__all__ = ["create_thruster_parameter_state"]

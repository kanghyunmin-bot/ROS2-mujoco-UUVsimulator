"""JSON loader for global and per-thruster tuning parameters."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, MutableMapping, Optional

from .thruster_param_apply import apply_thruster_param_payload
from .thruster_param_payload import read_thruster_param_payload
from .thruster_param_per_thruster import reset_per_thruster_params


def load_thruster_params_file(
    path: Path,
    thruster_names: Iterable[str],
    *,
    thruster_global: MutableMapping[str, Any],
    thruster_scale: MutableMapping[str, float],
    thruster_direct_scale: MutableMapping[str, float],
    thruster_reverse_asymmetry: MutableMapping[str, Optional[float]],
    thruster_tau_up: MutableMapping[str, Optional[float]],
    thruster_tau_down: MutableMapping[str, Optional[float]],
) -> bool:
    """Load global and per-thruster tuning parameters from JSON."""
    names = list(thruster_names)
    reset_per_thruster_params(
        names,
        thruster_scale=thruster_scale,
        thruster_direct_scale=thruster_direct_scale,
        thruster_reverse_asymmetry=thruster_reverse_asymmetry,
        thruster_tau_up=thruster_tau_up,
        thruster_tau_down=thruster_tau_down,
    )
    if not path.exists():
        return False
    payload = read_thruster_param_payload(path)
    if payload is None:
        return False

    return apply_thruster_param_payload(
        payload,
        thruster_global=thruster_global,
        thruster_scale=thruster_scale,
        thruster_direct_scale=thruster_direct_scale,
        thruster_reverse_asymmetry=thruster_reverse_asymmetry,
        thruster_tau_up=thruster_tau_up,
        thruster_tau_down=thruster_tau_down,
    )


__all__ = ["load_thruster_params_file", "reset_per_thruster_params"]

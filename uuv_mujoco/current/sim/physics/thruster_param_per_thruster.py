"""Per-thruster parameter parsing helpers."""

from __future__ import annotations

from typing import Any, Iterable, MutableMapping, Optional

from .thruster_param_common import is_number
from .thruster_param_gain_apply import clipped_gain_product, update_thruster_gain
from .thruster_param_optional_apply import apply_optional_thruster_overrides


def reset_per_thruster_params(
    thruster_names: Iterable[str],
    *,
    thruster_scale: MutableMapping[str, float],
    thruster_direct_scale: MutableMapping[str, float],
    thruster_reverse_asymmetry: MutableMapping[str, Optional[float]],
    thruster_tau_up: MutableMapping[str, Optional[float]],
    thruster_tau_down: MutableMapping[str, Optional[float]],
) -> None:
    """Reset mutable per-thruster maps before applying a JSON payload."""
    for name in thruster_names:
        thruster_scale[name] = 1.0
        thruster_direct_scale[name] = 1.0
        thruster_reverse_asymmetry[name] = None
        thruster_tau_up[name] = None
        thruster_tau_down[name] = None


def apply_one_thruster_config(
    name: str,
    cfg: dict[str, Any],
    *,
    global_gain_scale: float,
    global_direct_gain_scale: float,
    thruster_scale: MutableMapping[str, float],
    thruster_direct_scale: MutableMapping[str, float],
    thruster_reverse_asymmetry: MutableMapping[str, Optional[float]],
    thruster_tau_up: MutableMapping[str, Optional[float]],
    thruster_tau_down: MutableMapping[str, Optional[float]],
) -> bool:
    changed = False
    gain = cfg.get("gain_scale", 1.0)
    if not is_number(gain):
        return False
    new_gain = clipped_gain_product(float(gain), global_gain_scale, lower=0.1)
    changed = update_thruster_gain(thruster_scale, name, new_gain) or changed

    direct_gain = cfg.get("direct_gain_scale", 1.0)
    if is_number(direct_gain):
        new_direct_gain = clipped_gain_product(float(direct_gain), global_direct_gain_scale, lower=0.001)
        changed = update_thruster_gain(thruster_direct_scale, name, new_direct_gain) or changed

    apply_optional_thruster_overrides(
        name,
        cfg,
        thruster_reverse_asymmetry=thruster_reverse_asymmetry,
        thruster_tau_up=thruster_tau_up,
        thruster_tau_down=thruster_tau_down,
    )
    return changed


__all__ = ["apply_one_thruster_config", "reset_per_thruster_params"]

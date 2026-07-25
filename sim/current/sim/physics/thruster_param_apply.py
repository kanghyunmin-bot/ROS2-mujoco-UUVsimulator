"""Apply parsed thruster parameter payloads to mutable runtime maps."""

from __future__ import annotations

from typing import Any, MutableMapping, Optional

from .thruster_param_global import apply_global_params
from .thruster_param_payload import global_thruster_payload, per_thruster_payload
from .thruster_param_per_thruster import apply_one_thruster_config


def apply_thruster_param_payload(
    payload: dict[str, Any],
    *,
    thruster_global: MutableMapping[str, Any],
    thruster_scale: MutableMapping[str, float],
    thruster_direct_scale: MutableMapping[str, float],
    thruster_reverse_asymmetry: MutableMapping[str, Optional[float]],
    thruster_tau_up: MutableMapping[str, Optional[float]],
    thruster_tau_down: MutableMapping[str, Optional[float]],
) -> bool:
    global_gain_scale, global_direct_gain_scale = apply_global_params(
        global_thruster_payload(payload),
        thruster_global,
    )
    per_thruster = per_thruster_payload(payload)
    if per_thruster is None:
        return False

    changed = False
    for name, cfg in per_thruster.items():
        if name not in thruster_scale or not isinstance(cfg, dict):
            continue
        changed = apply_one_thruster_config(
            name,
            cfg,
            global_gain_scale=global_gain_scale,
            global_direct_gain_scale=global_direct_gain_scale,
            thruster_scale=thruster_scale,
            thruster_direct_scale=thruster_direct_scale,
            thruster_reverse_asymmetry=thruster_reverse_asymmetry,
            thruster_tau_up=thruster_tau_up,
            thruster_tau_down=thruster_tau_down,
        ) or changed
    return changed


__all__ = ["apply_thruster_param_payload"]

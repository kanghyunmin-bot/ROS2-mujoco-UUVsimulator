"""Global thruster parameter parsing helpers."""

from __future__ import annotations

from typing import Any, MutableMapping

from .thruster_param_common import is_number
from .thruster_param_global_fields import (
    apply_global_poly_fields,
    apply_global_scalar_fields,
    clipped_global_scale,
)


def apply_global_params(global_cfg: dict[str, Any], thruster_global: MutableMapping[str, Any]) -> tuple[float, float]:
    apply_global_scalar_fields(global_cfg, thruster_global)

    scale_all = global_cfg.get("gain_scale_all")
    global_gain_scale = clipped_global_scale(scale_all, default=1.0, lower=0.1, upper=20.0)
    if is_number(scale_all):
        thruster_global["gain_scale_all"] = global_gain_scale

    direct_scale_all = global_cfg.get("direct_gain_scale_all")
    global_direct_gain_scale = clipped_global_scale(
        direct_scale_all,
        default=1.0,
        lower=0.001,
        upper=20.0,
    )
    if is_number(direct_scale_all):
        thruster_global["direct_gain_scale_all"] = global_direct_gain_scale

    apply_global_poly_fields(global_cfg, thruster_global)
    return global_gain_scale, global_direct_gain_scale


__all__ = ["apply_global_params"]

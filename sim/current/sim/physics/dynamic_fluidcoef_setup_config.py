"""Profile parsing for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DynamicFluidcoefProfileConfig:
    cfg: dict
    enabled: bool
    reference_geom_scales: dict
    coefficient_load_weights: dict
    coefficient_axis_weights: dict
    coefficient_angular_axis_weights: dict
    min_multiplier: float
    max_multiplier: float
    default_weights: np.ndarray


def _dict_section(cfg: dict, name: str) -> dict:
    section = cfg.get(name)
    return section if isinstance(section, dict) else {}


def load_dynamic_fluidcoef_profile_config(
    *,
    sim_profile: dict,
    fluid_model: str,
    fluid_geom_ids: np.ndarray,
    env_flag: Callable[[str, bool], bool],
    to_float_array: Callable[[object], np.ndarray | None],
) -> DynamicFluidcoefProfileConfig:
    cfg = sim_profile.get("dynamic_fluidcoef")
    if not isinstance(cfg, dict):
        cfg = {}
    enabled = bool(
        env_flag("UUV_DYNAMIC_FLUIDCOEF_ENABLE", bool(cfg.get("active", False)))
        and str(fluid_model) == "current"
        and fluid_geom_ids.size
    )
    min_multiplier = float(np.clip(float(cfg.get("min_multiplier", 0.25)), 0.01, 100.0))
    max_multiplier = float(np.clip(float(cfg.get("max_multiplier", 25.0)), 0.01, 100.0))
    default_weights = to_float_array(cfg.get("default_load_weights", [1, 1, 1, 1, 1]))
    if default_weights is None or default_weights.size != 5:
        default_weights = np.ones(5, dtype=np.float64)
    default_weights = np.clip(default_weights.astype(np.float64, copy=False), 0.0, 5.0)
    return DynamicFluidcoefProfileConfig(
        cfg=cfg,
        enabled=enabled,
        reference_geom_scales=_dict_section(cfg, "reference_geom_scales"),
        coefficient_load_weights=_dict_section(cfg, "coefficient_load_weights"),
        coefficient_axis_weights=_dict_section(cfg, "coefficient_axis_weights"),
        coefficient_angular_axis_weights=_dict_section(cfg, "coefficient_angular_axis_weights"),
        min_multiplier=min_multiplier,
        max_multiplier=max_multiplier,
        default_weights=default_weights,
    )


__all__ = ["DynamicFluidcoefProfileConfig", "load_dynamic_fluidcoef_profile_config"]

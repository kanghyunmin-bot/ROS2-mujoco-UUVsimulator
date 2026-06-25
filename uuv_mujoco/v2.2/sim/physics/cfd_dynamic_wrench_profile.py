"""Profile and environment parsing for CFD-derived dynamic wrench runtime."""

from __future__ import annotations

from typing import Any, Callable, Optional

import numpy as np

from sim.physics.cfd_dynamic_wrench_axis_parser import parse_cfd_dynamic_wrench_axes
from sim.physics.cfd_dynamic_wrench_types import CfdDynamicWrenchRuntime


def build_cfd_dynamic_wrench_runtime(
    sim_profile: dict[str, Any],
    *,
    env_flag: Callable[[str, bool], bool],
    env_float: Callable[[str, float], float],
    to_float_array: Callable[[Any], Optional[np.ndarray]],
    log: Optional[Callable[[str], None]] = None,
) -> CfdDynamicWrenchRuntime:
    """Build runtime CFD dynamic-wrench settings from profile and environment."""
    cfg = _profile_config(sim_profile)
    enabled = bool(env_flag("UUV_CFD_DYNAMIC_WRENCH_ENABLE", bool(cfg.get("active", False))))
    scale = _runtime_scale(cfg, env_float)
    debug = env_flag("UUV_CFD_DYNAMIC_WRENCH_DEBUG", bool(cfg.get("debug", False)))
    axes = parse_cfd_dynamic_wrench_axes(cfg, to_float_array=to_float_array, log=log) if enabled else {}
    enabled = bool(enabled and axes and scale > 1.0e-12)
    source = str(cfg.get("source", "profile"))
    if enabled and log is not None:
        axis_text = ", ".join(sorted(axes))
        log(
            "[physics] CFD dynamic wrench active: "
            f"axes={axis_text}, scale={scale:.3f}, source={source}"
        )
    return CfdDynamicWrenchRuntime(
        enabled=enabled,
        scale=scale,
        debug=debug,
        axes=axes,
        source=source,
    )


def _profile_config(sim_profile: dict[str, Any]) -> dict[str, Any]:
    cfg = sim_profile.get("cfd_dynamic_wrench", {})
    return cfg if isinstance(cfg, dict) else {}


def _runtime_scale(cfg: dict[str, Any], env_float: Callable[[str, float], float]) -> float:
    return float(
        np.clip(
            env_float(
                "UUV_CFD_DYNAMIC_WRENCH_SCALE",
                float(cfg.get("scale", 1.0)),
            ),
            0.0,
            10.0,
        )
    )


__all__ = ["build_cfd_dynamic_wrench_runtime"]

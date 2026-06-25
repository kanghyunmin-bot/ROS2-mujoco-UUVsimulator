"""Enable-state helpers for dynamic MuJoCo fluid coefficient setup."""

from __future__ import annotations

from sim.physics.dynamic_fluidcoef_setup_config import DynamicFluidcoefProfileConfig
from sim.physics.dynamic_fluidcoef_setup_logging import (
    log_dynamic_fluidcoef_disabled_no_geoms,
    log_dynamic_fluidcoef_enabled,
)


def dynamic_fluidcoef_enabled_after_patterns(
    *,
    profile_cfg: DynamicFluidcoefProfileConfig,
    active_geom_ids: set[int],
) -> bool:
    if not profile_cfg.enabled:
        return False
    if not active_geom_ids:
        log_dynamic_fluidcoef_disabled_no_geoms()
        return False
    log_dynamic_fluidcoef_enabled(profile_cfg.cfg)
    return True


__all__ = ["dynamic_fluidcoef_enabled_after_patterns"]

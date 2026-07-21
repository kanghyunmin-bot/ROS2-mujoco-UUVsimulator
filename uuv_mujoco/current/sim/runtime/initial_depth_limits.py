"""Environment-derived limits for automatic initial Bar30 depth."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass


EnvFloat = Callable[[str, float], float]


@dataclass(frozen=True)
class AutoInitialDepthLimits:
    margin_m: float
    sitl_hold_start_depth_m: float
    min_depth_m: float
    surface_clear_m: float

    def seed_candidates(self) -> list[tuple[str, float]]:
        return [
            ("surface_depth_clearance", self.surface_clear_m),
            ("minimum_requested_depth", self.min_depth_m),
            ("sitl_hold_start_depth", self.sitl_hold_start_depth_m),
        ]


def load_auto_initial_depth_limits(*, sitl: bool, env_float: EnvFloat) -> AutoInitialDepthLimits:
    margin_m = float(max(env_float("UUV_INITIAL_SUBMERGED_MARGIN_M", 0.08), 0.0))
    sitl_hold_start_depth_m = float(
        max(env_float("UUV_INITIAL_BAR30_HOLD_START_DEPTH_M", 0.0), 0.0)
    )
    min_depth_m = float(max(env_float("UUV_INITIAL_BAR30_MIN_DEPTH_M", 0.60 if sitl else 0.0), 0.0))
    surface_depth_m = abs(env_float("SITL_SURFACE_DEPTH", -10.0)) / 100.0
    return AutoInitialDepthLimits(
        margin_m=margin_m,
        sitl_hold_start_depth_m=sitl_hold_start_depth_m,
        min_depth_m=min_depth_m,
        surface_clear_m=surface_depth_m + 0.05,
    )


__all__ = ["AutoInitialDepthLimits", "EnvFloat", "load_auto_initial_depth_limits"]

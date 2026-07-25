"""Drop-start default handling for SITL initial state."""

from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any


def apply_drop_start_default(*, args: Any, env_float: Callable[[str, float], float]) -> None:
    """Apply the SITL drop-start default when explicitly enabled."""

    drop_start_enabled_raw = os.environ.get("UUV_SITL_DROP_START_ABOVE_WATER", "0").strip().lower()
    drop_start_enabled = drop_start_enabled_raw in {"1", "true", "yes", "on", "enable", "enabled"}
    if (
        args.sitl
        and args.initial_depth_m is None
        and args.initial_bar30_depth_m is None
        and drop_start_enabled
    ):
        drop_height_m = float(max(env_float("UUV_SITL_DROP_HEIGHT_M", 0.35), 0.0))
        args.initial_depth_m = -drop_height_m


__all__ = ["apply_drop_start_default"]

"""Apply initial Bar30/base-depth requests to MuJoCo base state."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from sim.runtime.initial_state_depth_logging import (
    log_bar30_site_missing_fallback,
    log_initial_bar30_depth_set,
    log_initial_base_depth_set,
)
from sim.runtime.initial_state_depths import InitialBar30Depth
from sim.runtime.initial_state_surface_warning import warn_initial_bar30_surface_hysteresis


def apply_initial_bar30_depth(
    *,
    args: Any,
    base_state: Any,
    initial_depth_hold: Any,
    initial_bar30: InitialBar30Depth,
    env_float: Callable[[str, float], float],
) -> None:
    target_bar30_depth_m = float(initial_bar30.value_m)
    base_state.set_bar30_depth(target_bar30_depth_m)
    log_initial_bar30_depth_set(
        base_state=base_state,
        initial_depth_hold=initial_depth_hold,
        initial_bar30=initial_bar30,
        target_bar30_depth_m=target_bar30_depth_m,
    )
    if base_state.bar30_site_id < 0:
        log_bar30_site_missing_fallback()
    warn_initial_bar30_surface_hysteresis(
        args=args,
        bar30_depth_m=base_state.bar30_depth_now_m(),
        env_float=env_float,
        message_tail="Increase --initial-bar30-depth-m before testing arm -> ALT_HOLD.",
    )


def apply_initial_base_depth(
    *,
    args: Any,
    base_state: Any,
    initial_depth_hold: Any,
    env_float: Callable[[str, float], float],
) -> None:
    initial_depth_m = float(args.initial_depth_m)
    base_state.set_base_depth(initial_depth_m)
    log_initial_base_depth_set(
        initial_depth_m=initial_depth_m,
        initial_depth_hold=initial_depth_hold,
    )
    if base_state.bar30_site_id < 0:
        return
    warn_initial_bar30_surface_hysteresis(
        args=args,
        bar30_depth_m=base_state.bar30_depth_now_m(),
        env_float=env_float,
        message_tail=(
            "ALT_HOLD may clamp upward heave and command a dive until it "
            "clears the surfaced state."
        ),
    )


__all__ = ["apply_initial_bar30_depth", "apply_initial_base_depth"]

"""Log formatting for initial Bar30/base-depth setup."""

from __future__ import annotations

from typing import Any

from sim.runtime.initial_state_depths import InitialBar30Depth


def log_initial_bar30_depth_set(
    *,
    base_state: Any,
    initial_depth_hold: Any,
    initial_bar30: InitialBar30Depth,
    target_bar30_depth_m: float,
) -> None:
    base_depth_m = base_state.water_surface_z - float(base_state.base_origin_world()[2])
    print(
        "[runtime] initial Bar30 depth set: "
        f"target={target_bar30_depth_m:.3f} m"
        + (f" ({initial_bar30.label})" if initial_bar30.label else "")
        + f" actual={base_state.bar30_depth_now_m():.3f} m "
        f"(base_link_depth={base_depth_m:.3f} m)"
        + (" with hold enabled" if initial_depth_hold["active"] else ""),
        flush=True,
    )


def log_bar30_site_missing_fallback() -> None:
    print(
        "[runtime] warning: bar30_site not found; --initial-bar30-depth-m "
        "fell back to base_link origin depth.",
        flush=True,
    )


def log_initial_base_depth_set(*, initial_depth_m: float, initial_depth_hold: Any) -> None:
    if initial_depth_m < 0.0:
        start_msg = f"drop start: base_link={-initial_depth_m:.3f} m above water"
    else:
        start_msg = f"initial depth set: {initial_depth_m:.3f} m"
    print(
        "[runtime] "
        + start_msg
        + (" with hold enabled" if initial_depth_hold["active"] else ""),
        flush=True,
    )


__all__ = [
    "log_bar30_site_missing_fallback",
    "log_initial_bar30_depth_set",
    "log_initial_base_depth_set",
]

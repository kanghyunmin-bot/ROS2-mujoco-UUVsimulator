"""Watched-parameter comparison helpers for closed-loop contract audits."""

from __future__ import annotations

from audit_closed_loop_param_watchlist import WATCH_PARAMS


def same_param_value(left: str | None, right: str | None) -> bool:
    if left is None or right is None:
        return left == right
    try:
        return abs(float(left) - float(right)) <= 1.0e-9
    except ValueError:
        return left == right


def watched_param_report(
    real_params: dict[str, str],
    sitl_params: dict[str, str],
) -> tuple[dict[str, str | None], dict[str, str | None], dict[str, dict[str, str | None]], list[str]]:
    watched_real = {key: real_params.get(key) for key in WATCH_PARAMS}
    watched_sitl = {key: sitl_params.get(key) for key in WATCH_PARAMS}
    mismatches = {
        key: {"real": watched_real.get(key), "sitl": watched_sitl.get(key)}
        for key in WATCH_PARAMS
        if watched_real.get(key) is not None
        and watched_sitl.get(key) is not None
        and not same_param_value(watched_real.get(key), watched_sitl.get(key))
    }
    missing_sitl = [
        key
        for key in WATCH_PARAMS
        if watched_real.get(key) is not None and watched_sitl.get(key) is None
    ]
    return watched_real, watched_sitl, mismatches, missing_sitl

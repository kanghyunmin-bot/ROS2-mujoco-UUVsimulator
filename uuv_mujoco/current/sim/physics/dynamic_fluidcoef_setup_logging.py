"""Logging helpers for dynamic MuJoCo fluid coefficient setup."""

from __future__ import annotations


def log_dynamic_fluidcoef_disabled_no_geoms() -> None:
    print(
        "[physics] dynamic MuJoCo fluidcoef disabled: no active reference geoms",
        flush=True,
    )


def log_dynamic_fluidcoef_enabled(cfg: dict) -> None:
    print(
        "[physics] dynamic MuJoCo fluidcoef enabled: "
        f"source={cfg.get('source', 'sim_profile')!r}, "
        "coeff=(blunt, slender, angular, Kutta, Magnus), "
        f"transient_mode={cfg.get('transient_mode', 'log_decay')!r}, "
        "log_decay_coefficients="
        f"{cfg.get('log_decay_coefficients', [True, True, True, False, False])}, "
        f"lift_smoothing_alpha={cfg.get('lift_smoothing_alpha', 0.25)}",
        flush=True,
    )


__all__ = ["log_dynamic_fluidcoef_disabled_no_geoms", "log_dynamic_fluidcoef_enabled"]

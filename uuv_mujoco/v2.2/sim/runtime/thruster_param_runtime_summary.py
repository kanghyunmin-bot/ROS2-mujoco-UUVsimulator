"""Runtime thruster parameter summary logging."""

from __future__ import annotations

from typing import Any, Callable

from sim.runtime.thruster_param_runtime_log_sections import (
    log_base_thruster_summary,
    log_direct_curve_overrides,
    log_yaw_dynamics_overrides,
    log_yaw_reverse_asymmetry_overrides,
)


def log_thruster_runtime_summary(
    *,
    global_params: dict[str, Any],
    direct_scale: dict[str, float],
    reverse_asymmetry: dict[str, float | None],
    tau_up: dict[str, float | None],
    tau_down: dict[str, float | None],
    perf_cfg: dict,
    all_thruster_names: list[str],
    yaw_thrusters: list[str],
    log: Callable[[str], None],
) -> None:
    log_base_thruster_summary(global_params=global_params, log=log)
    log_direct_curve_overrides(
        perf_cfg=perf_cfg,
        direct_scale=direct_scale,
        all_thruster_names=all_thruster_names,
        log=log,
    )
    log_yaw_reverse_asymmetry_overrides(
        global_params=global_params,
        reverse_asymmetry=reverse_asymmetry,
        yaw_thrusters=yaw_thrusters,
        log=log,
    )
    log_yaw_dynamics_overrides(
        global_params=global_params,
        tau_up=tau_up,
        tau_down=tau_down,
        yaw_thrusters=yaw_thrusters,
        log=log,
    )


__all__ = [
    "log_base_thruster_summary",
    "log_direct_curve_overrides",
    "log_thruster_runtime_summary",
    "log_yaw_dynamics_overrides",
    "log_yaw_reverse_asymmetry_overrides",
]

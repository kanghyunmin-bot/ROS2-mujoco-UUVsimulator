"""Individual thruster parameter runtime summary log sections."""

from __future__ import annotations

from typing import Any, Callable

from sim.runtime.thruster_param_runtime_yaw_logs import (
    log_yaw_dynamics_overrides,
    log_yaw_reverse_asymmetry_overrides,
)


def log_base_thruster_summary(*, global_params: dict[str, Any], log: Callable[[str], None]) -> None:
    params = global_params
    log(
        "[thruster] dynamics: "
        f"deadzone={params['deadzone']:.3f}, "
        f"tau_up={params['tau_up']:.3f}s, "
        f"tau_down={params['tau_down']:.3f}s, "
        f"reverse_asym={params['reverse_asymmetry']:.3f}, "
        f"command_limit={params['command_limit']:.3f}, "
        f"reaction_tau_gain={params['reaction_torque_gain']:.4f}, "
        f"gain_scale_all={params['gain_scale_all']:.3f}, "
        f"direct_gain_scale_all={params['direct_gain_scale_all']:.3f}"
    )


def log_direct_curve_overrides(
    *,
    perf_cfg: dict,
    direct_scale: dict[str, float],
    all_thruster_names: list[str],
    log: Callable[[str], None],
) -> None:
    if not (perf_cfg.get("active") and perf_cfg.get("direct")):
        return
    direct_scale_overrides = [
        f"{name}={float(direct_scale[name]):.3f}"
        for name in all_thruster_names
        if abs(float(direct_scale[name]) - 1.0) > 1e-8
    ]
    if direct_scale_overrides:
        log("[thruster perf] direct curve gain overrides: " + ", ".join(direct_scale_overrides))


__all__ = [
    "log_base_thruster_summary",
    "log_direct_curve_overrides",
    "log_yaw_dynamics_overrides",
    "log_yaw_reverse_asymmetry_overrides",
]

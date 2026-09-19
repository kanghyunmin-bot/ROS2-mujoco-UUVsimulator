"""File loading and override application for runtime thruster parameters."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Callable

from sim.physics.thruster_params import (
    apply_thruster_direct_gain_overrides,
    ensure_thruster_params_file,
    load_thruster_params_file,
)
from sim.physics.thruster_inflow import thruster_inflow_profile_overrides


def load_runtime_thruster_parameters(
    *,
    path: Path,
    thruster_names: list[str],
    global_params: dict[str, Any],
    scale: dict[str, float],
    direct_scale: dict[str, float],
    reverse_asymmetry: dict[str, float | None],
    tau_up: dict[str, float | None],
    tau_down: dict[str, float | None],
    sim_profile: dict,
    vertical_thrusters: list[str],
    horizontal_thrusters: list[str],
    env_get: Callable[[str, str], str],
    log: Callable[[str], None],
) -> None:
    ensure_thruster_params_file(path, thruster_names)
    load_thruster_params_file(
        path,
        thruster_names,
        thruster_global=global_params,
        thruster_scale=scale,
        thruster_direct_scale=direct_scale,
        thruster_reverse_asymmetry=reverse_asymmetry,
        thruster_tau_up=tau_up,
        thruster_tau_down=tau_down,
    )
    inflow_override = thruster_inflow_profile_overrides(sim_profile)
    if inflow_override is not None:
        values, status, provenance = inflow_override
        global_params.update(values)
        log(
            "[thruster] local inflow correction: "
            f"{'on' if bool(values['inflow_enabled']) else 'off'} "
            f"(status={status}, provenance={provenance})"
        )
    apply_thruster_direct_gain_overrides(
        sim_profile,
        direct_scale,
        vertical_thrusters=vertical_thrusters,
        horizontal_thrusters=horizontal_thrusters,
        env_get=env_get,
        log=log,
    )
    apply_thruster_tau_profile_overrides(
        sim_profile=sim_profile,
        horizontal_thrusters=horizontal_thrusters,
        tau_up=tau_up,
        tau_down=tau_down,
        log=log,
    )
    apply_thruster_tau_env_overrides(
        thruster_names=thruster_names,
        vertical_thrusters=vertical_thrusters,
        horizontal_thrusters=horizontal_thrusters,
        tau_up=tau_up,
        tau_down=tau_down,
        env_get=env_get,
        log=log,
    )


def apply_thruster_tau_profile_overrides(
    *,
    sim_profile: dict,
    horizontal_thrusters: list[str],
    tau_up: dict[str, float | None],
    tau_down: dict[str, float | None],
    log: Callable[[str], None],
) -> None:
    """Apply profile-owned horizontal drive response time constants [s]."""
    raw = sim_profile.get("thruster_response_time_constants")
    if raw is None:
        return
    if not isinstance(raw, dict) or set(raw) != {
        "calibration_status",
        "provenance",
        "horizontal",
    }:
        raise ValueError(
            "thruster_response_time_constants requires status, provenance, and horizontal values"
        )
    if any(
        not isinstance(raw[key], str) or not raw[key].strip()
        for key in ("calibration_status", "provenance")
    ):
        raise ValueError(
            "thruster response calibration status and provenance must be non-empty"
        )
    horizontal = raw["horizontal"]
    if not isinstance(horizontal, dict) or set(horizontal) != {
        "tau_up_s",
        "tau_down_s",
    }:
        raise ValueError(
            "horizontal thruster response requires tau_up_s and tau_down_s"
        )
    up, down = float(horizontal["tau_up_s"]), float(horizontal["tau_down_s"])
    if not all(math.isfinite(value) and value > 0.0 for value in (up, down)):
        raise ValueError(
            "thruster response time constants must be finite and positive [s]"
        )
    _apply_tau_override(
        names=horizontal_thrusters,
        label=f"profile horizontal ({raw['calibration_status']})",
        up=up,
        down=down,
        tau_up=tau_up,
        tau_down=tau_down,
        log=lambda message: log(
            message.replace("tau env override:", "tau profile override:")
        ),
    )


def apply_thruster_tau_env_overrides(
    *,
    thruster_names: list[str],
    vertical_thrusters: list[str],
    horizontal_thrusters: list[str],
    tau_up: dict[str, float | None],
    tau_down: dict[str, float | None],
    env_get: Callable[[str, str], str],
    log: Callable[[str], None],
) -> None:
    all_up = _first_tau_env(env_get, "UUV_THRUSTER_TAU_UP", "UUV_THRUSTER_TAU_UP_ALL")
    all_down = _first_tau_env(
        env_get, "UUV_THRUSTER_TAU_DOWN", "UUV_THRUSTER_TAU_DOWN_ALL"
    )
    _apply_tau_override(
        "all",
        thruster_names,
        up=all_up,
        down=all_down,
        tau_up=tau_up,
        tau_down=tau_down,
        log=log,
    )

    vertical_up = _first_tau_env(env_get, "UUV_VERTICAL_THRUSTER_TAU_UP")
    vertical_down = _first_tau_env(env_get, "UUV_VERTICAL_THRUSTER_TAU_DOWN")
    _apply_tau_override(
        "vertical",
        vertical_thrusters,
        up=vertical_up,
        down=vertical_down,
        tau_up=tau_up,
        tau_down=tau_down,
        log=log,
    )

    yaw_up = _first_tau_env(
        env_get, "UUV_YAW_THRUSTER_TAU_UP", "UUV_HORIZONTAL_THRUSTER_TAU_UP"
    )
    yaw_down = _first_tau_env(
        env_get,
        "UUV_YAW_THRUSTER_TAU_DOWN",
        "UUV_HORIZONTAL_THRUSTER_TAU_DOWN",
    )
    _apply_tau_override(
        "yaw",
        horizontal_thrusters,
        up=yaw_up,
        down=yaw_down,
        tau_up=tau_up,
        tau_down=tau_down,
        log=log,
    )


def _first_tau_env(env_get: Callable[[str, str], str], *keys: str) -> float | None:
    for key in keys:
        value = _tau_env(env_get, key)
        if value is not None:
            return value
    return None


def _tau_env(env_get: Callable[[str, str], str], key: str) -> float | None:
    raw = env_get(key, "")
    text = "" if raw is None else str(raw).strip()
    if not text:
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    if not math.isfinite(value):
        return None
    return float(min(max(value, 1.0e-4), 2.0))


def _apply_tau_override(
    label: str,
    names: list[str],
    *,
    up: float | None,
    down: float | None,
    tau_up: dict[str, float | None],
    tau_down: dict[str, float | None],
    log: Callable[[str], None],
) -> None:
    if not names or (up is None and down is None):
        return
    for name in names:
        if name not in tau_up or name not in tau_down:
            continue
        if up is not None:
            tau_up[name] = up
        if down is not None:
            tau_down[name] = down
    up_text = "unchanged" if up is None else f"{up:.4f}s"
    down_text = "unchanged" if down is None else f"{down:.4f}s"
    log(f"[thruster] {label} tau env override: up={up_text}, down={down_text}")


__all__ = [
    "apply_thruster_tau_env_overrides",
    "apply_thruster_tau_profile_overrides",
    "load_runtime_thruster_parameters",
]

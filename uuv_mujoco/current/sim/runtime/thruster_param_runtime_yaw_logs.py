"""Yaw-specific thruster runtime summary log sections."""

from __future__ import annotations

from typing import Any, Callable


def log_yaw_reverse_asymmetry_overrides(
    *,
    global_params: dict[str, Any],
    reverse_asymmetry: dict[str, float | None],
    yaw_thrusters: list[str],
    log: Callable[[str], None],
) -> None:
    params = global_params
    yaw_reverse_asym = [
        reverse_asymmetry[name] if reverse_asymmetry[name] is not None else params["reverse_asymmetry"]
        for name in yaw_thrusters
    ]
    if _has_yaw_reverse_asymmetry_override(yaw_reverse_asym, default=params["reverse_asymmetry"]):
        log(
            "[thruster] yaw reverse asym override: "
            + ", ".join(f"{name}={float(value):.3f}" for name, value in zip(yaw_thrusters, yaw_reverse_asym))
        )


def log_yaw_dynamics_overrides(
    *,
    global_params: dict[str, Any],
    tau_up: dict[str, float | None],
    tau_down: dict[str, float | None],
    yaw_thrusters: list[str],
    log: Callable[[str], None],
) -> None:
    params = global_params
    yaw_tau_pairs = [
        (
            tau_up[name] if tau_up[name] is not None else params["tau_up"],
            tau_down[name] if tau_down[name] is not None else params["tau_down"],
        )
        for name in yaw_thrusters
    ]
    if _has_yaw_tau_override(
        yaw_tau_pairs,
        default_tau_up=params["tau_up"],
        default_tau_down=params["tau_down"],
    ):
        log(
            "[thruster] yaw dynamics override: "
            + ", ".join(
                f"{name}=up{float(up):.3f}/down{float(down):.3f}s"
                for name, (up, down) in zip(yaw_thrusters, yaw_tau_pairs)
            )
        )


def _has_yaw_reverse_asymmetry_override(values: list[float], *, default: float) -> bool:
    return bool(
        values
        and (
            len(set(round(float(value), 6) for value in values)) > 1
            or abs(float(values[0]) - float(default)) > 1e-8
        )
    )


def _has_yaw_tau_override(
    values: list[tuple[float, float]],
    *,
    default_tau_up: float,
    default_tau_down: float,
) -> bool:
    return any(
        abs(float(up) - float(default_tau_up)) > 1e-8
        or abs(float(down) - float(default_tau_down)) > 1e-8
        for up, down in values
    )


__all__ = ["log_yaw_dynamics_overrides", "log_yaw_reverse_asymmetry_overrides"]

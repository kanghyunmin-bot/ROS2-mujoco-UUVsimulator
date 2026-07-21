"""Hydrodynamic coefficient value and logging helpers."""

from __future__ import annotations

import math
from collections.abc import Callable


def read_yaw_torque_scale(*, hydro_cfg, env_float: Callable[[str, float], float]) -> tuple[float, float]:
    yaw_torque_scale_config = env_float(
        "UUV_YAW_TORQUE_SCALE",
        float(max(hydro_cfg.yaw_torque_scale, 0.0)),
    )
    return yaw_torque_scale_config, float(max(yaw_torque_scale_config, 0.0))


def read_yaw_torque_thruster_scales(
    *,
    sim_profile: dict,
    env_float: Callable[[str, float], float] | None = None,
) -> dict[str, float]:
    if env_float is not None and env_float("UUV_YAW_TORQUE_THRUSTER_SCALES_ENABLE", 1.0) <= 0.0:
        return {}
    raw = sim_profile.get("yaw_torque_thruster_scales", {})
    if not isinstance(raw, dict):
        return {}
    scales: dict[str, float] = {}
    for name, value in raw.items():
        try:
            scale = float(value)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(scale):
            continue
        scales[str(name)] = float(max(0.0, min(scale, 12.0)))
    return scales


def read_extra_hydro_coefficients(
    *,
    sim_profile: dict,
    env_float: Callable[[str, float], float],
) -> tuple[float, float, float, float, float, float, float, float, float]:
    hydro_pitch_moment_coeff = env_float(
        "UUV_HYDRO_PITCH_MOMENT_COEFF",
        float(sim_profile.get("hydro_pitch_moment_coeff", 0.0)),
    )
    hydro_vertical_lift_coeff = env_float(
        "UUV_HYDRO_VERTICAL_LIFT_COEFF",
        float(sim_profile.get("hydro_vertical_lift_coeff", 0.0)),
    )
    hydro_vertical_lift_deadband_mps = float(
        max(
            env_float(
                "UUV_HYDRO_VERTICAL_LIFT_DEADBAND_MPS",
                float(sim_profile.get("hydro_vertical_lift_deadband_mps", 0.0)),
            ),
            0.0,
        )
    )
    hydro_vertical_lift_power = float(
        max(
            env_float(
                "UUV_HYDRO_VERTICAL_LIFT_POWER",
                float(sim_profile.get("hydro_vertical_lift_power", 2.0)),
            ),
            1.0,
        )
    )
    hydro_yawrate_heave_pos_coeff = env_float(
        "UUV_HYDRO_YAWRATE_HEAVE_POS_COEFF",
        float(sim_profile.get("hydro_yawrate_heave_pos_coeff", 0.0)),
    )
    hydro_yawrate_heave_neg_coeff = env_float(
        "UUV_HYDRO_YAWRATE_HEAVE_NEG_COEFF",
        float(sim_profile.get("hydro_yawrate_heave_neg_coeff", 0.0)),
    )
    hydro_yawrate_heave_speed_deadband_mps = float(
        max(
            env_float(
                "UUV_HYDRO_YAWRATE_HEAVE_SPEED_DEADBAND_MPS",
                float(sim_profile.get("hydro_yawrate_heave_speed_deadband_mps", 0.0)),
            ),
            0.0,
        )
    )
    hydro_yawrate_heave_yaw_deadband_radps = float(
        max(
            env_float(
                "UUV_HYDRO_YAWRATE_HEAVE_YAW_DEADBAND_RADPS",
                float(sim_profile.get("hydro_yawrate_heave_yaw_deadband_radps", 0.0)),
            ),
            0.0,
        )
    )
    heave_extra_damping_n_per_mps = float(
        max(
            env_float(
                "UUV_HEAVE_EXTRA_DAMPING_N_PER_MPS",
                float(sim_profile.get("heave_extra_damping_n_per_mps", 0.0)),
            ),
            0.0,
        )
    )
    return (
        hydro_pitch_moment_coeff,
        hydro_vertical_lift_coeff,
        hydro_vertical_lift_deadband_mps,
        hydro_vertical_lift_power,
        hydro_yawrate_heave_pos_coeff,
        hydro_yawrate_heave_neg_coeff,
        hydro_yawrate_heave_speed_deadband_mps,
        hydro_yawrate_heave_yaw_deadband_radps,
        heave_extra_damping_n_per_mps,
    )


def log_yaw_scale_override(yaw_torque_scale: float, log: Callable[[str], None]) -> None:
    if abs(yaw_torque_scale - 1.0) > 1.0e-9:
        log(
            "[thruster] yaw torque scale: "
            f"UUV_YAW_TORQUE_SCALE={yaw_torque_scale:.4f}"
        )


def log_yaw_torque_thruster_scales(
    yaw_torque_thruster_scales: dict[str, float],
    log: Callable[[str], None],
) -> None:
    if not yaw_torque_thruster_scales:
        return
    formatted = ", ".join(
        f"{name}={value:.3f}" for name, value in sorted(yaw_torque_thruster_scales.items())
    )
    log("[thruster] per-yaw-thruster yaw torque scales: " + formatted)


def log_extra_hydro_coefficients(
    *,
    hydro_pitch_moment_coeff: float,
    hydro_vertical_lift_coeff: float,
    hydro_vertical_lift_deadband_mps: float,
    hydro_vertical_lift_power: float,
    hydro_yawrate_heave_pos_coeff: float,
    hydro_yawrate_heave_neg_coeff: float,
    hydro_yawrate_heave_speed_deadband_mps: float,
    hydro_yawrate_heave_yaw_deadband_radps: float,
    heave_extra_damping_n_per_mps: float,
    log: Callable[[str], None],
) -> None:
    if abs(hydro_pitch_moment_coeff) > 1.0e-9:
        log(
            "[physics] hydrodynamic pitch moment: "
            f"coeff={hydro_pitch_moment_coeff:+.4f} N*m/(m/s)^2"
        )
    if abs(hydro_vertical_lift_coeff) > 1.0e-9:
        log(
            "[physics] hydrodynamic vertical lift: "
            f"coeff={hydro_vertical_lift_coeff:+.4f}, "
            f"deadband={hydro_vertical_lift_deadband_mps:.3f}m/s, "
            f"power={hydro_vertical_lift_power:.3f}"
        )
    if (
        abs(hydro_yawrate_heave_pos_coeff) > 1.0e-9
        or abs(hydro_yawrate_heave_neg_coeff) > 1.0e-9
    ):
        log(
            "[physics] yaw-rate heave coupling: "
            f"pos_coeff={hydro_yawrate_heave_pos_coeff:+.4f}, "
            f"neg_coeff={hydro_yawrate_heave_neg_coeff:+.4f}, "
            f"speed_deadband={hydro_yawrate_heave_speed_deadband_mps:.3f}m/s, "
            f"yaw_deadband={hydro_yawrate_heave_yaw_deadband_radps:.3f}rad/s"
        )
    if heave_extra_damping_n_per_mps > 1.0e-9:
        log(
            "[physics] heave extra damping: "
            f"coeff={heave_extra_damping_n_per_mps:.4f} N/(m/s)"
        )


__all__ = [
    "log_extra_hydro_coefficients",
    "log_yaw_scale_override",
    "log_yaw_torque_thruster_scales",
    "read_extra_hydro_coefficients",
    "read_yaw_torque_scale",
    "read_yaw_torque_thruster_scales",
]

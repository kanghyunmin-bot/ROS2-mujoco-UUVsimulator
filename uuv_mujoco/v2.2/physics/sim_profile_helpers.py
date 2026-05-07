"""Helpers for simulation profile loading and physics knob grouping."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from .hydrodynamics_helpers import estimate_ellipsoid_hydrodynamics


PROFILE_ALIASES: dict[str, str] = {
    "legacy": "legacy",
    "custom": "legacy",
    "current": "current",
    "ellipsoid": "current",
}


def canonical_profile_name(name: str) -> str:
    key = str(name).strip().lower()
    return PROFILE_ALIASES.get(key, key)


DEFAULT_SIM_PROFILES: dict[str, dict[str, Any]] = {
    "legacy": {
        "half_height": 0.147,
        "buoyancy_model": "ellipsoid",
        "buoyancy_scale": 0.9997,
        "buoyancy_slope_scale": 1.0,
        "surface_heave_damping": 18.0,
        "heave_damping_scale": 1.0,
        "cob_torque_scale": 0.20,
        "buoyancy_point_blend": 1.0,
        "cob_x_offset": -0.003,
        "cob_z_offset": 0.010,
        "thruster_voltage": 20.0,
        "thruster_force_max": 65.0,
        "linear_drag": 1.10,
        "angular_drag": 0.32,
        "linear_damping_linear": [1.10, 1.32, 1.54],
        "linear_damping_angular": [0.32, 0.36, 0.28],
        "quadratic_damping_linear": [1.40, 2.00, 2.40],
        "quadratic_damping_angular": [0.10, 0.12, 0.08],
        "added_mass_linear": [1.80, 2.30, 3.00],
        "added_mass_angular": [0.10, 0.12, 0.08],
        "current_world": [0.0, 0.0, 0.0],
        "body_components": [
            {
                "name": "center_enclosure",
                "shape": "ellipsoid",
                "size": [0.220, 0.120, 0.110],
                "mass": 6.0,
                "mass_pos": [-0.005, 0.0, 0.075],
                "buoyancy_pos": [-0.005, 0.0, 0.060],
                "buoyancy_share": 0.20,
            },
            {
                "name": "port_lower_body",
                "shape": "ellipsoid",
                "size": [0.380, 0.090, 0.090],
                "mass": 4.5,
                "mass_pos": [0.0, 0.208, -0.005],
                "buoyancy_pos": [0.0, 0.208, 0.040],
                "buoyancy_share": 0.40,
            },
            {
                "name": "starboard_lower_body",
                "shape": "ellipsoid",
                "size": [0.380, 0.090, 0.090],
                "mass": 4.5,
                "mass_pos": [0.0, -0.208, -0.005],
                "buoyancy_pos": [0.0, -0.208, 0.040],
                "buoyancy_share": 0.40,
            },
        ],
        "buoyancy_points": [
            {
                "name": "buoy_ver_lf",
                "pos": [0.149, 0.2346, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
            {
                "name": "buoy_ver_lr",
                "pos": [-0.171, 0.2346, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
            {
                "name": "buoy_ver_rf",
                "pos": [0.149, -0.2344, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
            {
                "name": "buoy_ver_rr",
                "pos": [-0.171, -0.2344, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
        ],
        "spin_gain": 22.0,
        "yaw_torque_scale": 1.0,
        "validation_timestep": 0.004,
        "validation_iterations": 20,
        "validation_ls_iterations": 8,
        "validation_step_start": 0.5,
        "validation_step_end": 1.4,
        "validation_total_time": 3.2,
        "validation_step_amp_ratio": 0.35,
    },
    "current": {
        "buoyancy_model": "ellipsoid",
        "buoyancy_scale": 1.0005,
        "buoyancy_slope_scale": 1.0,
        "surface_heave_damping": 18.0,
        "heave_damping_scale": 1.0,
        "cob_torque_scale": 1.05,
        "buoyancy_point_blend": 1.0,
        "cob_x_offset": 0.0,
        "cob_z_offset": 0.012,
        "thruster_voltage": 16.0,
        "thruster_force_max": 21.0,
        "linear_drag": 1.75,
        "angular_drag": 0.60,
        "current_world": [0.0, 0.0, 0.0],
        "body_components": [
            {
                "name": "center_enclosure",
                "shape": "ellipsoid",
                "size": [0.220, 0.120, 0.110],
                "mass": 6.0,
                "mass_pos": [-0.005, 0.0, 0.075],
                "buoyancy_pos": [-0.005, 0.0, 0.060],
                "buoyancy_share": 0.20,
            },
            {
                "name": "port_lower_body",
                "shape": "ellipsoid",
                "size": [0.380, 0.090, 0.090],
                "mass": 4.5,
                "mass_pos": [0.0, 0.208, -0.005],
                "buoyancy_pos": [0.0, 0.208, 0.040],
                "buoyancy_share": 0.40,
            },
            {
                "name": "starboard_lower_body",
                "shape": "ellipsoid",
                "size": [0.380, 0.090, 0.090],
                "mass": 4.5,
                "mass_pos": [0.0, -0.208, -0.005],
                "buoyancy_pos": [0.0, -0.208, 0.040],
                "buoyancy_share": 0.40,
            },
        ],
        "buoyancy_points": [
            {
                "name": "buoy_ver_lf",
                "pos": [0.149, 0.2346, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
            {
                "name": "buoy_ver_lr",
                "pos": [-0.171, 0.2346, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
            {
                "name": "buoy_ver_rf",
                "pos": [0.149, -0.2344, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
            {
                "name": "buoy_ver_rr",
                "pos": [-0.171, -0.2344, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
        ],
        "ellipsoid_model": {
            "active": True,
            "semi_axes": [0.160, 0.106, 0.147],
            "use_shape_volume": False,
            "effective_cd_linear": [0.105, 0.095, 0.165],
            "effective_cd_angular": [3.80, 1.75, 2.35],
            "added_mass_scale_linear": [0.52, 0.32, 0.72],
            "added_mass_scale_angular": [1.30, 1.15, 1.25],
            "linear_damping_ratio_linear": 2.4,
            "linear_damping_ratio_angular": 6.0,
            "reference_speed_linear": 0.30,
            "reference_speed_angular": 0.60,
        },
        "spin_gain": 22.0,
        "yaw_torque_scale": 1.0,
        "validation_timestep": 0.005,
        "validation_iterations": 20,
        "validation_ls_iterations": 8,
        "validation_step_start": 0.5,
        "validation_step_end": 1.4,
        "validation_total_time": 3.2,
        "validation_step_amp_ratio": 0.30,
    },
}


@dataclass(frozen=True)
class HydrodynamicsConfig:
    """Physics knobs that are commonly tuned together for underwater feel."""

    half_height: float
    buoyancy_model: str
    buoyancy_scale: float
    buoyancy_slope_scale: float
    surface_heave_damping: float
    heave_damping_scale: float
    cob_torque_scale: float
    buoyancy_point_blend: float
    thruster_force_max: float
    linear_drag: float
    angular_drag: float
    air_linear_drag: float
    air_angular_drag: float
    spin_gain: float
    yaw_torque_scale: float
    added_mass_diag: np.ndarray
    linear_damping_diag: np.ndarray
    quadratic_damping_diag: np.ndarray
    air_linear_damping_diag: np.ndarray
    water_current_world: np.ndarray
    displaced_volume: float | None
    model_source: str
    ellipsoid_semi_axes: np.ndarray | None
    body_components: tuple["BodyComponent", ...]
    buoyancy_points: tuple["BuoyancyPoint", ...]


@dataclass(frozen=True)
class BodyComponent:
    """Intuitive component-level mass/buoyancy definition for one rigid body."""

    name: str
    shape: str
    size: np.ndarray
    mass: float
    mass_pos: np.ndarray
    buoyancy_pos: np.ndarray
    buoyancy_share: float


@dataclass(frozen=True)
class BuoyancyPoint:
    """Distributed buoyancy application point."""

    name: str
    pos: np.ndarray
    share: float
    half_height: float


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _normalize_buoyancy_model(value: Any) -> str:
    name = str(value).strip().lower()
    if name in {"linear", "ellipsoid"}:
        return name
    return "ellipsoid"


def _to_float_array(value: Any, size: int) -> np.ndarray | None:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        return None
    try:
        out = np.array([float(item) for item in value], dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(out)):
        return None
    return out


def _normalize_component_shape(value: Any) -> str:
    name = str(value).strip().lower()
    if name == "box":
        return "box"
    return "ellipsoid"


def _parse_buoyancy_points(
    sim_profile: Mapping[str, Any],
    default_half_height: float,
) -> tuple[BuoyancyPoint, ...]:
    payload = sim_profile.get("buoyancy_points")
    if not isinstance(payload, list):
        return ()

    points: list[BuoyancyPoint] = []
    for idx, item in enumerate(payload):
        if not isinstance(item, Mapping):
            continue
        pos = _to_float_array(item.get("pos"), 3)
        if pos is None:
            continue
        try:
            share = float(item.get("share", 0.0))
            half_height = float(item.get("half_height", default_half_height))
        except (TypeError, ValueError):
            continue
        if not np.isfinite(share) or share < 0.0:
            continue
        if not np.isfinite(half_height) or half_height <= 0.0:
            continue
        name = str(item.get("name", f"buoyancy_point_{idx}")).strip() or f"buoyancy_point_{idx}"
        points.append(
            BuoyancyPoint(
                name=name,
                pos=pos.astype(np.float64, copy=True),
                share=share,
                half_height=half_height,
            )
        )
    return tuple(points)


def _parse_body_components(sim_profile: Mapping[str, Any]) -> tuple[BodyComponent, ...]:
    payload = sim_profile.get("body_components")
    if not isinstance(payload, list):
        return ()

    components: list[BodyComponent] = []
    for idx, item in enumerate(payload):
        if not isinstance(item, Mapping):
            continue
        size = _to_float_array(item.get("size"), 3)
        if size is None or np.any(size <= 0.0):
            continue
        mass_pos = _to_float_array(item.get("mass_pos"), 3)
        if mass_pos is None:
            mass_pos = _to_float_array(item.get("pos"), 3)
        if mass_pos is None:
            continue
        buoyancy_pos = _to_float_array(item.get("buoyancy_pos"), 3)
        if buoyancy_pos is None:
            buoyancy_pos = mass_pos.copy()
        try:
            mass = float(item.get("mass", 0.0))
            buoyancy_share = float(item.get("buoyancy_share", mass))
        except (TypeError, ValueError):
            continue
        if not np.isfinite(mass) or mass <= 0.0:
            continue
        if not np.isfinite(buoyancy_share) or buoyancy_share < 0.0:
            continue
        name = str(item.get("name", f"component_{idx}")).strip() or f"component_{idx}"
        components.append(
            BodyComponent(
                name=name,
                shape=_normalize_component_shape(item.get("shape", "ellipsoid")),
                size=size.astype(np.float64, copy=True),
                mass=mass,
                mass_pos=mass_pos.astype(np.float64, copy=True),
                buoyancy_pos=buoyancy_pos.astype(np.float64, copy=True),
                buoyancy_share=buoyancy_share,
            )
        )
    return tuple(components)


def _vector_from_keys(
    payload: Mapping[str, Any],
    direct_key: str,
    split_prefix: str,
    default_linear: np.ndarray,
    default_angular: np.ndarray | None = None,
) -> np.ndarray:
    direct = _to_float_array(payload.get(direct_key), 6)
    if direct is not None:
        return direct

    linear = _to_float_array(payload.get(f"{split_prefix}_linear"), 3)
    if linear is None:
        linear = np.array(default_linear, dtype=np.float64)

    if default_angular is None:
        return linear

    angular = _to_float_array(payload.get(f"{split_prefix}_angular"), 3)
    if angular is None:
        angular = np.array(default_angular, dtype=np.float64)

    return np.concatenate((linear, angular)).astype(np.float64, copy=False)


def _vector3_from_keys(payload: Mapping[str, Any], *keys: str, default: tuple[float, float, float]) -> np.ndarray:
    for key in keys:
        values = _to_float_array(payload.get(key), 3)
        if values is not None:
            return values
    return np.array(default, dtype=np.float64)


def _resolve_ellipsoid_baseline(
    sim_profile: Mapping[str, Any],
    fluid_density: float,
) -> tuple[dict[str, Any] | None, str]:
    payload = sim_profile.get("ellipsoid_model")
    if not isinstance(payload, Mapping) or not bool(payload.get("active", False)):
        return None, "coefficient-profile"

    semi_axes = _to_float_array(payload.get("semi_axes"), 3)
    if semi_axes is None or np.any(semi_axes <= 0.0):
        return None, "coefficient-profile"

    estimate = estimate_ellipsoid_hydrodynamics(
        semi_axes,
        fluid_density,
        effective_cd_linear=_vector3_from_keys(
            payload,
            "effective_cd_linear",
            "quadratic_cd_linear",
            default=(0.10, 0.11, 0.13),
        ),
        effective_cd_angular=_vector3_from_keys(
            payload,
            "effective_cd_angular",
            "quadratic_cd_angular",
            default=(2.2, 2.4, 1.8),
        ),
        added_mass_scale_linear=_vector3_from_keys(
            payload,
            "added_mass_scale_linear",
            default=(1.0, 1.0, 1.0),
        ),
        added_mass_scale_angular=_vector3_from_keys(
            payload,
            "added_mass_scale_angular",
            default=(1.2, 1.2, 1.0),
        ),
        linear_damping_ratio_linear=float(payload.get("linear_damping_ratio_linear", 2.0)),
        linear_damping_ratio_angular=float(payload.get("linear_damping_ratio_angular", 1.0)),
        reference_speed_linear=float(payload.get("reference_speed_linear", 0.30)),
        reference_speed_angular=float(payload.get("reference_speed_angular", 0.75)),
    )
    return {
        "semi_axes": estimate.semi_axes,
        "half_height": float(payload.get("half_height", estimate.semi_axes[2])),
        "displaced_volume": (
            estimate.displaced_volume if bool(payload.get("use_shape_volume", True)) else None
        ),
        "added_mass_diag": estimate.added_mass_diag,
        "linear_damping_diag": estimate.linear_damping_diag,
        "quadratic_damping_diag": estimate.quadratic_damping_diag,
    }, "ellipsoid-baseline"


def load_sim_profiles(profile_path: Path) -> tuple[dict[str, dict[str, Any]], str | None]:
    """Load profile overrides from JSON and merge with built-in defaults."""

    profile_path = Path(profile_path).expanduser()
    if not profile_path.exists():
        profile_path.write_text(json.dumps(DEFAULT_SIM_PROFILES, indent=2))

    profiles = {name: dict(cfg) for name, cfg in DEFAULT_SIM_PROFILES.items()}
    try:
        payload = json.loads(profile_path.read_text())
    except json.JSONDecodeError:
        return profiles, f"[profile] invalid json: {profile_path}, using built-in defaults"

    if isinstance(payload, dict):
        for name, cfg in payload.items():
            if not isinstance(cfg, dict):
                continue
            canonical_name = canonical_profile_name(name)
            merged = dict(profiles.get(canonical_name, {}))
            merged.update(cfg)
            profiles[canonical_name] = merged
    return profiles, None


def build_sim_profile(
    profiles: Mapping[str, Mapping[str, Any]],
    profile_name: str,
    buoyancy_scale_override: float | None = None,
) -> dict[str, Any]:
    """Return one resolved profile with optional runtime overrides applied."""

    resolved_name = canonical_profile_name(profile_name)
    if resolved_name not in profiles:
        raise KeyError(profile_name)
    sim_profile = dict(profiles[resolved_name])
    if buoyancy_scale_override is not None:
        sim_profile["buoyancy_scale"] = _clamp(float(buoyancy_scale_override), 0.0, 2.0)
    return sim_profile


def build_hydrodynamics_config(
    sim_profile: Mapping[str, Any],
    perf_force_max: float | None = None,
    fluid_density: float = 1000.0,
) -> HydrodynamicsConfig:
    """Group the main underwater dynamics knobs into one object."""

    linear_drag = float(sim_profile.get("linear_drag", 1.2))
    angular_drag = float(sim_profile.get("angular_drag", 0.12))
    thruster_force_max = float(sim_profile.get("thruster_force_max", 50.0))
    if perf_force_max is not None and float(perf_force_max) > 0.0:
        thruster_force_max = float(perf_force_max)

    ellipsoid_defaults, model_source = _resolve_ellipsoid_baseline(sim_profile, fluid_density)
    default_linear_linear = (
        np.array(ellipsoid_defaults["linear_damping_diag"][:3], dtype=np.float64)
        if ellipsoid_defaults is not None
        else np.full(3, linear_drag, dtype=np.float64)
    )
    default_linear_angular = (
        np.array(ellipsoid_defaults["linear_damping_diag"][3:], dtype=np.float64)
        if ellipsoid_defaults is not None
        else np.full(3, angular_drag, dtype=np.float64)
    )
    default_quadratic_linear = (
        np.array(ellipsoid_defaults["quadratic_damping_diag"][:3], dtype=np.float64)
        if ellipsoid_defaults is not None
        else np.zeros(3, dtype=np.float64)
    )
    default_quadratic_angular = (
        np.array(ellipsoid_defaults["quadratic_damping_diag"][3:], dtype=np.float64)
        if ellipsoid_defaults is not None
        else np.zeros(3, dtype=np.float64)
    )
    default_added_mass_linear = (
        np.array(ellipsoid_defaults["added_mass_diag"][:3], dtype=np.float64)
        if ellipsoid_defaults is not None
        else np.zeros(3, dtype=np.float64)
    )
    default_added_mass_angular = (
        np.array(ellipsoid_defaults["added_mass_diag"][3:], dtype=np.float64)
        if ellipsoid_defaults is not None
        else np.zeros(3, dtype=np.float64)
    )

    linear_damping_diag = _vector_from_keys(
        sim_profile,
        direct_key="linear_damping_diag",
        split_prefix="linear_damping",
        default_linear=default_linear_linear,
        default_angular=default_linear_angular,
    )
    quadratic_damping_diag = _vector_from_keys(
        sim_profile,
        direct_key="quadratic_damping_diag",
        split_prefix="quadratic_damping",
        default_linear=default_quadratic_linear,
        default_angular=default_quadratic_angular,
    )
    added_mass_diag = _vector_from_keys(
        sim_profile,
        direct_key="added_mass_diag",
        split_prefix="added_mass",
        default_linear=default_added_mass_linear,
        default_angular=default_added_mass_angular,
    )
    air_linear_damping_diag = np.concatenate(
        (
            np.full(3, _clamp(float(sim_profile.get("air_linear_drag", linear_drag * 0.03)), 0.0, linear_drag)),
            np.full(3, _clamp(float(sim_profile.get("air_angular_drag", angular_drag * 0.05)), 0.0, angular_drag)),
        )
    ).astype(np.float64, copy=False)
    water_current_world = _vector3_from_keys(
        sim_profile,
        "water_current_world",
        "current_world",
        default=(0.0, 0.0, 0.0),
    )

    default_half_height = float(
        sim_profile.get(
            "half_height",
            ellipsoid_defaults["half_height"] if ellipsoid_defaults is not None else 0.147,
        )
    )

    return HydrodynamicsConfig(
        half_height=default_half_height,
        buoyancy_model=_normalize_buoyancy_model(sim_profile.get("buoyancy_model", "ellipsoid")),
        buoyancy_scale=float(sim_profile.get("buoyancy_scale", 1.0)),
        buoyancy_slope_scale=max(float(sim_profile.get("buoyancy_slope_scale", 1.0)), 0.1),
        surface_heave_damping=max(float(sim_profile.get("surface_heave_damping", 0.0)), 0.0),
        heave_damping_scale=max(float(sim_profile.get("heave_damping_scale", 1.0)), 0.0),
        cob_torque_scale=float(sim_profile.get("cob_torque_scale", 1.0)),
        buoyancy_point_blend=_clamp(float(sim_profile.get("buoyancy_point_blend", 1.0)), 0.0, 1.0),
        thruster_force_max=thruster_force_max,
        linear_drag=linear_drag,
        angular_drag=angular_drag,
        air_linear_drag=float(air_linear_damping_diag[0]),
        air_angular_drag=float(air_linear_damping_diag[3]),
        spin_gain=float(sim_profile.get("spin_gain", 22.0)),
        yaw_torque_scale=float(sim_profile.get("yaw_torque_scale", 1.0)),
        added_mass_diag=added_mass_diag,
        linear_damping_diag=linear_damping_diag,
        quadratic_damping_diag=quadratic_damping_diag,
        air_linear_damping_diag=air_linear_damping_diag,
        water_current_world=water_current_world,
        displaced_volume=(
            None if ellipsoid_defaults is None else ellipsoid_defaults["displaced_volume"]
        ),
        model_source=model_source,
        ellipsoid_semi_axes=(
            None if ellipsoid_defaults is None else ellipsoid_defaults["semi_axes"].copy()
        ),
        body_components=_parse_body_components(sim_profile),
        buoyancy_points=_parse_buoyancy_points(sim_profile, default_half_height),
    )

"""Predicate checks for the dynamic MuJoCo fluidcoef source contract."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all


def _profile_schema_ok(paths: dict[str, Path]) -> bool:
    return contains_all(
        paths["sim_profiles_json"],
        [
            '"dynamic_fluidcoef": {',
            '"active": false',
            '"update_hz": 20.0',
            '"default_load_weights"',
            '"coefficient_axis_weights"',
            '"coefficient_angular_axis_weights"',
            '"reference_geom_scales"',
        ],
    )


def _setup_path_ok(paths: dict[str, Path]) -> bool:
    return contains_all(
        paths["model_setup_py"],
        [
            "fluid_geom_ids, fluid_geom_names, fluidcoef_static_geom_scales = apply_fluid_geom_runtime_scales(",
            "fluidcoef_dynamic_setup = build_dynamic_fluidcoef_setup(",
            "fluidcoef_dynamic_setup=fluidcoef_dynamic_setup",
        ],
    ) and contains_all(
        paths["setup_config_py"],
        [
            'env_flag("UUV_DYNAMIC_FLUIDCOEF_ENABLE"',
            'str(fluid_model) == "current"',
            "and fluid_geom_ids.size",
            "default_load_weights",
        ],
    )


def _pattern_path_ok(paths: dict[str, Path]) -> bool:
    return (
        contains_all(
            paths["setup_enable_py"],
            [
                "dynamic_fluidcoef_enabled_after_patterns",
                "if not active_geom_ids:",
                "log_dynamic_fluidcoef_enabled(profile_cfg.cfg)",
            ],
        )
        and contains_all(
            paths["setup_rows_py"],
            [
                "prepare_dynamic_fluidcoef_pattern",
                "apply_dynamic_fluidcoef_pattern_arrays",
                "active_geom_ids",
            ],
        )
        and contains_all(
            paths["pattern_prepare_py"],
            [
                "DynamicFluidcoefPatternSetup",
                "reference_scale.size != 5",
                "np.clip(ratio, profile_cfg.min_multiplier, profile_cfg.max_multiplier)",
                "matching_fluid_geom_ids(fluid_geom_names, pattern)",
            ],
        )
    )


def _load_math_ok(paths: dict[str, Path]) -> bool:
    return contains_all(
        paths["loads_py"],
        [
            "fluidcoef_loads_from_local_velocity",
            "axis_loads = np.clip(np.abs(rel) / ref_speed, 0.0, 1.0)",
            "angular_loads = np.clip(np.abs(omega) / ref_angular, 0.0, 1.0)",
            "reference_speed_mps",
            "reference_angular_rps",
            "MuJoCo fluidcoef order: blunt, slender, angular, Kutta, Magnus.",
        ],
    )


def _runtime_update_ok(paths: dict[str, Path]) -> bool:
    return (
        contains_all(
            paths["runtime_config_py"],
            [
                "configure_dynamic_fluidcoef_update_knobs(runtime, env_float=env_float)",
                "configure_dynamic_fluidcoef_transient_knobs(",
                "initialize_dynamic_fluidcoef_runtime_buffers(runtime, env_flag=env_flag)",
            ],
        )
        and contains_all(
            paths["runtime_update_py"],
            [
                "dynamic_fluidcoef_update_due",
                "runtime.next_sim_t += runtime.update_dt",
                "mj_objectVelocity",
                "current_local = geom_rot.T @ runtime.water_current_world",
                "np.clip(runtime.weights[int(geom_id), :] * coeff_loads, 0.0, 1.0)",
                "runtime.model.geom_fluid[idx, 1:6]",
            ],
        )
        and contains_all(
            paths["runtime_py"],
            [
                "class DynamicFluidcoefRuntime",
                "def update",
                "compute_dynamic_fluidcoef_blend",
            ],
        )
    )


def _flow_owner_ok(paths: dict[str, Path]) -> bool:
    return contains_all(
        paths["current_py"],
        [
            "model.opt.wind[:] = water_current_world",
            "MuJoCo's built-in ellipsoid fluid model uses opt.wind",
        ],
    ) and contains_all(
        paths["underwater_runtime_py"],
        [
            "self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)",
            "apply_hydrodynamic_wrenches(",
            "Apply the single-owner underwater wrench",
        ],
    )


def dynamic_fluidcoef_contract_ok(paths: dict[str, Path]) -> bool:
    return all(
        (
            _profile_schema_ok(paths),
            _setup_path_ok(paths),
            _pattern_path_ok(paths),
            _load_math_ok(paths),
            _runtime_update_ok(paths),
            _flow_owner_ok(paths),
        )
    )


__all__ = ["dynamic_fluidcoef_contract_ok"]

"""Evidence builders for the dynamic MuJoCo fluidcoef source contract."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import evidence
from audit_code_contract_types import Evidence


def build_dynamic_fluidcoef_evidence(paths: dict[str, Path]) -> list[Evidence]:
    return [
        evidence(paths["sim_profiles_json"], '"dynamic_fluidcoef": {'),
        evidence(paths["sim_profiles_json"], '"active": false'),
        evidence(paths["model_setup_py"], "fluidcoef_dynamic_setup = build_dynamic_fluidcoef_setup("),
        evidence(paths["setup_config_py"], 'env_flag("UUV_DYNAMIC_FLUIDCOEF_ENABLE"'),
        evidence(paths["setup_config_py"], 'str(fluid_model) == "current"'),
        evidence(paths["setup_enable_py"], "dynamic_fluidcoef_enabled_after_patterns"),
        evidence(paths["setup_rows_py"], "active_geom_ids"),
        evidence(paths["pattern_prepare_py"], "reference_scale.size != 5"),
        evidence(paths["pattern_prepare_py"], "matching_fluid_geom_ids(fluid_geom_names, pattern)"),
        evidence(paths["loads_py"], "MuJoCo fluidcoef order: blunt, slender, angular, Kutta, Magnus."),
        evidence(paths["loads_py"], "axis_loads = np.clip(np.abs(rel) / ref_speed, 0.0, 1.0)"),
        evidence(paths["runtime_config_py"], "configure_dynamic_fluidcoef_update_knobs(runtime, env_float=env_float)"),
        evidence(paths["runtime_update_py"], "runtime.next_sim_t += runtime.update_dt"),
        evidence(paths["runtime_update_py"], "current_local = geom_rot.T @ runtime.water_current_world"),
        evidence(paths["runtime_update_py"], "np.clip(runtime.weights[int(geom_id), :] * coeff_loads, 0.0, 1.0)"),
        evidence(paths["runtime_update_py"], "runtime.model.geom_fluid[idx, 1:6]"),
        evidence(paths["runtime_py"], "class DynamicFluidcoefRuntime"),
        evidence(paths["current_py"], "model.opt.wind[:] = water_current_world"),
        evidence(paths["underwater_runtime_py"], "self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)"),
    ]


__all__ = ["build_dynamic_fluidcoef_evidence"]

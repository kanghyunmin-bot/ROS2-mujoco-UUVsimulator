"""Active-runtime dynamic MuJoCo fluidcoef contract source check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_runtime_dynamic_fluidcoef_eval import dynamic_fluidcoef_contract_ok
from audit_code_contract_runtime_dynamic_fluidcoef_evidence import build_dynamic_fluidcoef_evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def _dynamic_fluidcoef_paths(runtime_paths: dict[str, Path]) -> dict[str, Path]:
    return {
        "sim_profiles_json": runtime_paths["sim_profiles_json"],
        "model_setup_py": runtime_paths["model_runtime_setup_py"],
        "setup_config_py": runtime_paths["dynamic_fluidcoef_setup_config_py"],
        "setup_enable_py": runtime_paths["dynamic_fluidcoef_setup_enable_py"],
        "setup_rows_py": runtime_paths["dynamic_fluidcoef_setup_rows_py"],
        "pattern_prepare_py": runtime_paths["dynamic_fluidcoef_pattern_prepare_py"],
        "loads_py": runtime_paths["dynamic_fluidcoef_loads_py"],
        "runtime_config_py": runtime_paths["dynamic_fluidcoef_runtime_config_py"],
        "runtime_update_py": runtime_paths["dynamic_fluidcoef_runtime_update_py"],
        "runtime_py": runtime_paths["dynamic_fluidcoef_runtime_py"],
        "current_py": runtime_paths["hydrodynamics_runtime_current_py"],
        "underwater_runtime_py": runtime_paths["underwater_wrench_runtime_py"],
    }


def build_runtime_dynamic_fluidcoef_contract_check(runtime_paths: dict[str, Path]) -> Check:
    paths = _dynamic_fluidcoef_paths(runtime_paths)
    ok = dynamic_fluidcoef_contract_ok(paths)
    return Check(
        check_id="active_runtime_dynamic_fluidcoef_contract",
        status="PASS" if ok else "FAIL",
        title="Dynamic ellipsoid fluidcoef changes are explicit velocity-load updates",
        conclusion=(
            "Dynamic MuJoCo fluidcoef is profile/env opt-in, restricted to current-mode fluid geoms, "
            "configured from pattern-specific five-coefficient reference rows, and updates the five "
            "MuJoCo ellipsoid coefficients from geom-local velocity and angular-rate loads. The accepted "
            "clean baseline keeps the dynamic path disabled until a validated HAN/CFD profile enables it."
        ),
        evidence=build_dynamic_fluidcoef_evidence(paths),
        official_refs=[OFFICIAL_REFS["mujoco_fluid"]],
    )


__all__ = ["build_runtime_dynamic_fluidcoef_contract_check"]

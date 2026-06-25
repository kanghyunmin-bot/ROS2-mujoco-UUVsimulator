"""Active MuJoCo runtime surface checks for source-level contract audits."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_runtime_atm_pressure import build_active_runtime_atm_pressure_exclusion_check
from audit_code_contract_runtime_baro import build_active_runtime_baro_contract_check
from audit_code_contract_runtime_json_altitude import build_active_runtime_json_altitude_check
from audit_code_contract_runtime_static_pressure import build_active_runtime_static_pressure_check
from audit_code_contract_types import Check


def build_active_runtime_surface_checks(runtime_paths: dict[str, Path], ardupilot_paths: dict[str, Path]) -> list[Check]:
    return [
        build_active_runtime_baro_contract_check(runtime_paths),
        build_active_runtime_json_altitude_check(runtime_paths, ardupilot_paths),
        build_active_runtime_static_pressure_check(runtime_paths),
        build_active_runtime_atm_pressure_exclusion_check(runtime_paths),
    ]

"""Extended active-runtime contract checks."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_runtime_dynamic_fluidcoef import (
    build_runtime_dynamic_fluidcoef_contract_check,
)
from audit_code_contract_runtime_flow_coupling import (
    build_runtime_flow_coupling_contract_check,
)
from audit_code_contract_runtime_plant_input import build_runtime_plant_input_contract_check
from audit_code_contract_runtime_rc import build_runtime_rc_in_out_contract_check
from audit_code_contract_runtime_sensor_io import build_runtime_sensor_io_contract_check
from audit_code_contract_runtime_time import build_runtime_time_contract_check
from audit_code_contract_types import Check


def build_extended_active_runtime_surface_checks(runtime_paths: dict[str, Path]) -> list[Check]:
    return [
        build_runtime_time_contract_check(runtime_paths),
        build_runtime_sensor_io_contract_check(runtime_paths),
        build_runtime_rc_in_out_contract_check(runtime_paths),
        build_runtime_plant_input_contract_check(runtime_paths),
        build_runtime_dynamic_fluidcoef_contract_check(runtime_paths),
        build_runtime_flow_coupling_contract_check(runtime_paths),
    ]


__all__ = [
    "build_extended_active_runtime_surface_checks",
    "build_runtime_dynamic_fluidcoef_contract_check",
    "build_runtime_flow_coupling_contract_check",
    "build_runtime_plant_input_contract_check",
    "build_runtime_rc_in_out_contract_check",
    "build_runtime_sensor_io_contract_check",
    "build_runtime_time_contract_check",
]

"""Build source-level contract checks for ArduSub and the active MuJoCo runtime."""

from __future__ import annotations

from audit_code_contract_common import ACTIVE_RUNTIME_ROOT, COMPAT_V22_ROOT, REPO_ROOT, load_contract_gate_summary
from audit_code_contract_matrix import build_contract_matrix_gate
from audit_code_contract_firmware_checks import build_json_baro_telemetry_checks, build_rc_input_checks
from audit_code_contract_paths import active_runtime_paths, ardupilot_source_paths, runtime_path
from audit_code_contract_runtime_checks import build_active_runtime_surface_checks
from audit_code_contract_runtime_surface_ext import build_extended_active_runtime_surface_checks
from audit_code_contract_source_identity import build_source_identity_checks
from audit_code_contract_thruster_gate_checks import build_thruster_and_gate_checks
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_checks() -> tuple[list[Check], dict[str, object]]:
    ardupilot_paths = ardupilot_source_paths()
    runtime_paths = active_runtime_paths()

    identity_checks, source_metadata = build_source_identity_checks()
    checks = [
        *identity_checks,
        *build_json_baro_telemetry_checks(ardupilot_paths),
        *build_rc_input_checks(ardupilot_paths),
        *build_active_runtime_surface_checks(runtime_paths, ardupilot_paths),
        *build_extended_active_runtime_surface_checks(runtime_paths),
        *build_thruster_and_gate_checks(runtime_paths, ardupilot_paths),
    ]
    checks.append(build_contract_matrix_gate(checks))

    gate_summary = load_contract_gate_summary()

    metadata = {
        "repo_root": str(REPO_ROOT),
        "active_runtime_root": str(ACTIVE_RUNTIME_ROOT),
        "compat_v22_root": str(COMPAT_V22_ROOT),
        **source_metadata,
        "official_refs": OFFICIAL_REFS,
        "contract_gate_summary": gate_summary,
    }
    return checks, metadata


__all__ = [
    "active_runtime_paths",
    "ardupilot_source_paths",
    "build_active_runtime_surface_checks",
    "build_checks",
    "build_extended_active_runtime_surface_checks",
    "build_json_baro_telemetry_checks",
    "build_rc_input_checks",
    "build_source_identity_checks",
    "build_thruster_and_gate_checks",
    "runtime_path",
]

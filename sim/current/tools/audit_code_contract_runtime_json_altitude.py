"""Active-runtime JSON altitude compatibility warning check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence, rel
from audit_code_contract_types import Check, Evidence, OFFICIAL_REFS


def build_active_runtime_json_altitude_check(
    runtime_paths: dict[str, Path],
    ardupilot_paths: dict[str, Path],
) -> Check:
    sim_json_h = ardupilot_paths["sim_json_h"]
    sitl_json_sensor_runtime_py = runtime_paths["sitl_json_sensor_runtime_py"]
    return Check(
        check_id="active_runtime_json_altitude_field_is_compat_only",
        status="WARN" if contains_all(sitl_json_sensor_runtime_py, ['"altitude": float(vertical_est.alt_m)']) else "PASS",
        title="Active runtime still sends JSON altitude but ArduSub 4.1.2 ignores it",
        conclusion=(
            "The altitude key in the active runtime JSON payload is compatibility/debug data for this firmware. "
            "It must not be treated as a controller input contract."
        ),
        evidence=[
            evidence(sitl_json_sensor_runtime_py, '"altitude": float(vertical_est.alt_m)'),
            Evidence(path=rel(sim_json_h), line=None, snippet="SIM_JSON keytable has no altitude key"),
        ],
        official_refs=[OFFICIAL_REFS["ardupilot_json_sitl"]],
    )


__all__ = ["build_active_runtime_json_altitude_check"]

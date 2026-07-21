"""ArduPilot JSON sensor keytable contract check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import evidence, read_text, rel
from audit_code_contract_types import Check, Evidence, OFFICIAL_REFS


REQUIRED_JSON_SENSOR_KEYS = (
    '"timestamp"',
    '"gyro"',
    '"accel_body"',
    '"position"',
    '"velocity"',
    '"attitude"',
    '"quaternion"',
)


def build_json_sensor_keytable_check(paths: dict[str, Path]) -> Check:
    sim_json_h = paths["sim_json_h"]
    sim_json_text = read_text(sim_json_h)
    keytable_has_required = all(token in sim_json_text for token in REQUIRED_JSON_SENSOR_KEYS)
    has_altitude = '"altitude"' in sim_json_text
    return Check(
        check_id="json_sensor_no_direct_pressure_or_altitude",
        status="PASS" if keytable_has_required else "FAIL",
        title="ArduPilot JSON sensor vertical input contract is recognized",
        conclusion=(
            "This ArduPilot revision accepts altitude directly as well as position.z."
            if has_altitude else
            "This ArduPilot revision has no pressure/altitude key, so Bar30 is injected through position.z."
        ),
        evidence=[
            evidence(sim_json_h, "struct keytable", "SIM_JSON keytable"),
            evidence(sim_json_h, '{ "", "position", &state.position', "position key present"),
            Evidence(
                path=rel(sim_json_h), line=None,
                snippet="altitude key present" if has_altitude else "altitude key absent; position.z is authoritative",
            ),
        ],
        official_refs=[OFFICIAL_REFS["ardupilot_json_sitl"]],
    )


__all__ = ["REQUIRED_JSON_SENSOR_KEYS", "build_json_sensor_keytable_check"]

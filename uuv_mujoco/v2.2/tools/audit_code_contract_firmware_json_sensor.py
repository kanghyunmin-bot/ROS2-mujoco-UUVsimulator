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
    keytable_has_pressure_or_altitude = '"pressure"' in sim_json_text or '"altitude"' in sim_json_text
    return Check(
        check_id="json_sensor_no_direct_pressure_or_altitude",
        status="PASS" if keytable_has_required and not keytable_has_pressure_or_altitude else "FAIL",
        title="ArduPilot JSON sensor parser has no pressure/altitude key",
        conclusion=(
            "Bar30 cannot be injected by a JSON pressure field in this firmware. "
            "The pressure contract must be implemented indirectly through position.z."
        ),
        evidence=[
            evidence(sim_json_h, '} keytable[16] = {', "SIM_JSON keytable[16]"),
            evidence(sim_json_h, '{ "", "position", &state.position', "position key present"),
            Evidence(path=rel(sim_json_h), line=None, snippet="pressure/altitude keys absent from keytable"),
        ],
        official_refs=[OFFICIAL_REFS["ardupilot_json_sitl"]],
    )


__all__ = ["REQUIRED_JSON_SENSOR_KEYS", "build_json_sensor_keytable_check"]

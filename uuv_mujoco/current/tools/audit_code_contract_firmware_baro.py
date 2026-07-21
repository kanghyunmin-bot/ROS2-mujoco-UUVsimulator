"""ArduSub SITL Bar30 pressure path contract check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_baro_from_json_position_check(paths: dict[str, Path]) -> Check:
    baro_sitl_cpp = paths["baro_sitl_cpp"]
    aircraft_cpp = paths["aircraft_cpp"]
    ok = contains_all(
        baro_sitl_cpp,
        [
            "BARO_TYPE_WATER",
            "float sim_alt = _sitl->state.altitude",
            "SimpleUnderWaterAtmosphere(-sim_alt * 0.001f",
        ],
    ) and contains_all(aircraft_cpp, ["location.alt", "home.alt - position.z * 100.0f"])
    return Check(
        check_id="baro_sitl_pressure_from_json_position_z",
        status="PASS" if ok else "FAIL",
        title="ArduSub SITL Bar30 pressure is derived from altitude built from position.z",
        conclusion=(
            "For ArduSub, AP_Baro_SITL registers a water barometer and converts SITL altitude "
            "to underwater pressure. SIM_Aircraft builds that altitude from JSON position.z."
        ),
        evidence=[
            evidence(baro_sitl_cpp, "BARO_TYPE_WATER"),
            evidence(baro_sitl_cpp, "float sim_alt = _sitl->state.altitude"),
            evidence(baro_sitl_cpp, "SimpleUnderWaterAtmosphere(-sim_alt * 0.001f"),
            evidence(aircraft_cpp, "location.alt  = static_cast<int32_t>(home.alt - position.z * 100.0f)"),
        ],
        official_refs=[OFFICIAL_REFS["bar30_pressure_sensor"]],
    )


__all__ = ["build_baro_from_json_position_check"]

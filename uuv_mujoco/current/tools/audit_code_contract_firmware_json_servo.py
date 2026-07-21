"""ArduPilot JSON servo packet contract check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_json_servo_packet_check(paths: dict[str, Path]) -> Check:
    sim_json_h = paths["sim_json_h"]
    sim_json_cpp = paths["sim_json_cpp"]
    ok = contains_all(sim_json_h, ["uint16_t pwm[16]"]) and contains_all(
        sim_json_cpp,
        ["for (uint8_t i=0; i<16; i++)", "pkt.pwm[i] = input.servos[i]"],
    )
    return Check(
        check_id="json_servo_packet_16_raw_pwm",
        status="PASS" if ok else "FAIL",
        title="SITL JSON servo backend is raw 16-channel PWM",
        conclusion="Closed-loop plant input must consume the raw JSON servo packet as PWM[0..15].",
        evidence=[
            evidence(sim_json_h, "uint16_t pwm[16]"),
            evidence(sim_json_cpp, "pkt.pwm[i] = input.servos[i]"),
        ],
        official_refs=[OFFICIAL_REFS["ardupilot_json_sitl"]],
    )


__all__ = ["build_json_servo_packet_check"]

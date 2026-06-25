"""ArduPilot RC override and joystick source checks."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_rc_input_checks(paths: dict[str, Path]) -> list[Check]:
    gcs_common_cpp = paths["gcs_common_cpp"]
    rc_channel_cpp = paths["rc_channel_cpp"]
    rc_varinfo_h = paths["rc_varinfo_h"]
    joystick_cpp = paths["joystick_cpp"]
    checks: list[Check] = []
    ok = contains_all(
        gcs_common_cpp,
        ["packet.chan16_raw", "for (uint8_t i=0; i<8; i++)", "for (uint8_t i=8; i<ARRAY_SIZE(override_data); i++)"],
    )
    checks.append(
        Check(
            check_id="rc_override_local_16_channel_limit",
            status="WARN" if ok else "FAIL",
            title="Local ArduSub 4.1.2 RC override handler consumes 1..16, not 1..18",
            conclusion=(
                "The official MAVLink message has extension channels beyond 16, but this local "
                "ArduSub 4.1.2 handler builds override_data only through chan16_raw. "
                "This is acceptable for the current vehicle if active controls stay within C1..C8, "
                "but the old 'preserve 1..18' checklist is not true for this firmware."
            ),
            evidence=[
                evidence(gcs_common_cpp, "packet.chan16_raw"),
                evidence(gcs_common_cpp, "for (uint8_t i=8; i<ARRAY_SIZE(override_data); i++)"),
            ],
            official_refs=[OFFICIAL_REFS["mavlink_rc_channels_override"]],
        )
    )

    ok = contains_all(rc_channel_cpp, ["last_override_time", "override_value = v", "get_override_timeout_ms"]) and contains_all(
        rc_varinfo_h,
        ['AP_GROUPINFO("_OVERRIDE_TIME"', "3.0"],
    )
    checks.append(
        Check(
            check_id="rc_override_timeout_policy",
            status="PASS" if ok else "FAIL",
            title="RC override has firmware timeout policy",
            conclusion=(
                "RC override must be streamed faster than RC_OVERRIDE_TIME, whose local default is 3s. "
                "A one-shot override is not a valid closed-loop input contract."
            ),
            evidence=[
                evidence(rc_channel_cpp, "override_value = v"),
                evidence(rc_channel_cpp, "get_override_timeout_ms"),
                evidence(rc_varinfo_h, 'AP_GROUPINFO("_OVERRIDE_TIME"'),
            ],
            official_refs=[OFFICIAL_REFS["mavlink_rc_channels_override"]],
        )
    )

    ok = contains_all(
        joystick_cpp,
        [
            "set_override(2",
            "set_override(3",
            "set_override(4",
            "set_override(5",
            "// throttle",
            "// yaw",
            "// forward for ROV",
            "// lateral for ROV",
        ],
    )
    checks.append(
        Check(
            check_id="ardusub_joystick_axis_mapping",
            status="PASS" if ok else "FAIL",
            title="ArduSub joystick maps RC3 heave, RC4 yaw, RC5 forward, RC6 lateral",
            conclusion="The real joy/RC override axis mapping checklist is confirmed in ArduSub code.",
            evidence=[
                evidence(joystick_cpp, "RC_Channels::set_override(2", "RC3 throttle/heave"),
                evidence(joystick_cpp, "RC_Channels::set_override(3", "RC4 yaw"),
                evidence(joystick_cpp, "RC_Channels::set_override(4", "RC5 forward"),
                evidence(joystick_cpp, "RC_Channels::set_override(5", "RC6 lateral"),
            ],
            official_refs=[],
        )
    )
    return checks


__all__ = ["build_rc_input_checks"]

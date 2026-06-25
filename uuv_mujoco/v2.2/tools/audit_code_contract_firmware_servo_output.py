"""MAVLink SERVO_OUTPUT_RAW telemetry contract check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_servo_output_raw_telemetry_check(paths: dict[str, Path]) -> Check:
    gcs_common_cpp = paths["gcs_common_cpp"]
    ok = contains_all(
        gcs_common_cpp,
        [
            "void GCS_MAVLINK::send_servo_output_raw()",
            "uint16_t values[16]",
            "hal.rcout->read(values, 16)",
            "mavlink_msg_servo_output_raw_send",
        ],
    )
    return Check(
        check_id="servo_output_raw_halrcout_telemetry",
        status="PASS" if ok else "FAIL",
        title="SERVO_OUTPUT_RAW is hal.rcout telemetry",
        conclusion=(
            "Controller parity comparison layer is real /mavros/rc/out versus SITL MAVLink "
            "SERVO_OUTPUT_RAW, not the high-rate JSON servo backend."
        ),
        evidence=[
            evidence(gcs_common_cpp, "void GCS_MAVLINK::send_servo_output_raw()"),
            evidence(gcs_common_cpp, "hal.rcout->read(values, 16)"),
        ],
        official_refs=[OFFICIAL_REFS["mavlink_servo_output_raw"]],
    )


__all__ = ["build_servo_output_raw_telemetry_check"]

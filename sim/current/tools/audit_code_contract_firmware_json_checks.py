"""ArduPilot JSON/Bar30/SERVO telemetry source checks."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_firmware_baro import build_baro_from_json_position_check
from audit_code_contract_firmware_json_sensor import build_json_sensor_keytable_check
from audit_code_contract_firmware_json_servo import build_json_servo_packet_check
from audit_code_contract_firmware_servo_output import build_servo_output_raw_telemetry_check
from audit_code_contract_types import Check


def build_json_baro_telemetry_checks(paths: dict[str, Path]) -> list[Check]:
    return [
        build_json_servo_packet_check(paths),
        build_json_sensor_keytable_check(paths),
        build_baro_from_json_position_check(paths),
        build_servo_output_raw_telemetry_check(paths),
    ]


__all__ = ["build_json_baro_telemetry_checks"]

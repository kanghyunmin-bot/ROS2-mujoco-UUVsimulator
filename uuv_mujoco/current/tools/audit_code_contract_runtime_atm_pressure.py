"""Active-runtime atm-pressure exclusion contract check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import evidence
from audit_code_contract_types import Check


def build_active_runtime_atm_pressure_exclusion_check(runtime_paths: dict[str, Path]) -> Check:
    ros2_publish_runtime_py = runtime_paths["ros2_publish_runtime_py"]
    return Check(
        check_id="active_runtime_atm_pressure_excluded_output_surface",
        status="WARN",
        title="/mavros/imu/atm_pressure is not a safe real-bag pressure target",
        conclusion=(
            "The April 1 real bag has /mavros/imu/atm_pressure around 0.24, not Pa-scale surface pressure. "
            "The sim currently publishes Bar30 absolute pressure there, so this topic is excluded from "
            "controller parity and plant replay fitting until its real semantics are proven."
        ),
        evidence=[evidence(ros2_publish_runtime_py, "mavros_atm_pressure_msg = build_pressure_msg")],
        official_refs=[],
    )


__all__ = ["build_active_runtime_atm_pressure_exclusion_check"]

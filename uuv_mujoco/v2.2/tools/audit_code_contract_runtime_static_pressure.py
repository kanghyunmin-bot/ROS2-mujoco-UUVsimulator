"""Active-runtime static-pressure ROS output contract check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_active_runtime_static_pressure_check(runtime_paths: dict[str, Path]) -> Check:
    ros2_bridge_config_static_pressure_py = runtime_paths["ros2_bridge_config_static_pressure_py"]
    ros2_publish_state_py = runtime_paths["ros2_publish_state_py"]
    ok = contains_all(
        ros2_bridge_config_static_pressure_py,
        [
            'ROS2_UUV_STATIC_PRESSURE_SOURCE", "external"',
        ],
    ) and contains_all(
        ros2_publish_state_py,
        [
            'if self._static_pressure_source == "external":',
            "static_pressure_pa = bar30_pressure_pa",
        ],
    )
    return Check(
        check_id="active_runtime_static_pressure_external_bar30",
        status="PASS" if ok else "FAIL",
        title="/mavros/imu/static_pressure defaults to external Bar30 pressure",
        conclusion=(
            "The ROS output surface uses Bar30 absolute pressure for static_pressure by default. "
            "For SITL control, the important path remains JSON position.z -> AP_Baro_SITL."
        ),
        evidence=[
            evidence(ros2_bridge_config_static_pressure_py, 'ROS2_UUV_STATIC_PRESSURE_SOURCE", "external"'),
            evidence(ros2_publish_state_py, "static_pressure_pa = bar30_pressure_pa"),
        ],
        official_refs=[OFFICIAL_REFS["bar30_pressure_sensor"]],
    )


__all__ = ["build_active_runtime_static_pressure_check"]

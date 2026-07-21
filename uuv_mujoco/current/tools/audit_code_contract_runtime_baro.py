"""Active-runtime Bar30 frontend-match contract check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_active_runtime_baro_contract_check(runtime_paths: dict[str, Path]) -> Check:
    sitl_contract_py = runtime_paths["sitl_contract_py"]
    baro_contract_py = runtime_paths["baro_contract_py"]
    sitl_json_sensor_runtime_py = runtime_paths["sitl_json_sensor_runtime_py"]
    ros2_state_estimation_py = runtime_paths["ros2_state_estimation_py"]
    ros2_state_sitl_vertical_py = runtime_paths["ros2_state_sitl_vertical_py"]
    ros2_state_sitl_baro_py = runtime_paths["ros2_state_sitl_baro_py"]
    ros2_bridge_config_baro_py = runtime_paths["ros2_bridge_config_baro_py"]
    ok = contains_all(
        baro_contract_py,
        ["AP_BARO_FRONTEND_PA_PER_M", "sitl_depth_m_for_frontend_match"],
    ) and contains_all(
        sitl_json_sensor_runtime_py,
        ["json_position[2] = float(vertical_est.depth_m)", '"position": [float(x) for x in json_position]'],
    ) and contains_all(
        ros2_bridge_config_baro_py,
        ['ROS2_UUV_SITL_BARO_DEPTH_CONTRACT", "frontend_match"'],
    ) and contains_all(
        ros2_state_sitl_baro_py,
        ["sitl_depth_m_for_frontend_match(pressure_pa)"],
    ) and contains_all(
        ros2_state_sitl_vertical_py,
        ["sitl_contract_depth_m("],
    ) and contains_all(
        ros2_state_estimation_py,
        ["from .ros2_state_vertical import"],
    )
    return Check(
        check_id="active_runtime_baro_contract_frontend_match",
        status="PASS" if ok else "FAIL",
        title="Active runtime injects Bar30 contract through JSON position.z frontend match",
        conclusion=(
            "The active runtime converts physical Bar30 pressure/depth into the JSON position.z value that makes "
            "AP_Baro_SITL produce the matching water-barometer frontend altitude."
        ),
        evidence=[
            evidence(baro_contract_py, "sitl_depth_m_for_frontend_match"),
            evidence(sitl_contract_py, "from sim.contracts.baro"),
            evidence(ros2_bridge_config_baro_py, 'ROS2_UUV_SITL_BARO_DEPTH_CONTRACT", "frontend_match"'),
            evidence(ros2_state_estimation_py, "from .ros2_state_vertical import"),
            evidence(ros2_state_sitl_vertical_py, "sitl_contract_depth_m("),
            evidence(ros2_state_sitl_baro_py, "sitl_depth_m_for_frontend_match(pressure_pa)"),
            evidence(sitl_json_sensor_runtime_py, "json_position[2] = float(vertical_est.depth_m)"),
        ],
        official_refs=[OFFICIAL_REFS["ardupilot_json_sitl"], OFFICIAL_REFS["bar30_pressure_sensor"]],
    )


__all__ = ["build_active_runtime_baro_contract_check"]

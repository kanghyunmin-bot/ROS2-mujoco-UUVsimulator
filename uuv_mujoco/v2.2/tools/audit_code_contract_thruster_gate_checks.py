"""Thruster and plant-replay gate checks for source-level contract audits."""

from __future__ import annotations

import json
from pathlib import Path

from audit_code_contract_common import contains_all, evidence, load_contract_gate_summary
from audit_code_contract_thruster_conversion import build_active_runtime_thruster_conversion_check
from audit_code_contract_types import Check, Evidence, OFFICIAL_REFS


def build_thruster_and_gate_checks(runtime_paths: dict[str, Path], ardupilot_paths: dict[str, Path]) -> list[Check]:
    motors6dof_cpp = ardupilot_paths["motors6dof_cpp"]
    thruster_mapping_py = runtime_paths["thruster_mapping_py"]
    checks: list[Check] = []
    ok = contains_all(
        motors6dof_cpp,
        [
            "SUB_FRAME_VECTORED_6DOF",
            "add_motor_raw_6dof(AP_MOTORS_MOT_1",
            "add_motor_raw_6dof(AP_MOTORS_MOT_8",
        ],
    ) and contains_all(
        thruster_mapping_py,
        [
            "ARDUSUB_VECTORED_6DOF_MOTOR_FACTORS_FRD",
            "REAL_ROBOT_MOT_DIRECTIONS",
            "ARDUSUB_VECTORED_6DOF_SERVO_MAP",
            "ARDUSUB_VECTORED_6DOF_SERVO_SIGNS",
        ],
    )
    checks.append(
        Check(
            check_id="thruster_contract_final_pwm_not_mot_direction_again",
            status="PASS" if ok else "FAIL",
            title="Plant maps final SERVO/JSON PWM once, without reapplying MOT_x_DIRECTION",
            conclusion=(
                "ArduSub motor factors and MOT_x_DIRECTION are controller-side. "
                "The active runtime maps final PWM into MuJoCo actuator-positive force using only physical mount orientation."
            ),
            evidence=[
                evidence(motors6dof_cpp, "case SUB_FRAME_VECTORED_6DOF:"),
                evidence(thruster_mapping_py, "REAL_ROBOT_MOT_DIRECTIONS"),
                evidence(thruster_mapping_py, "ARDUSUB_VECTORED_6DOF_SERVO_SIGNS"),
            ],
            official_refs=[],
        )
    )
    checks.append(build_active_runtime_thruster_conversion_check(runtime_paths))

    gate_summary = load_contract_gate_summary()
    checks.append(
        Check(
            check_id="plant_replay_gate_safe_targets",
            status="WARN" if gate_summary.get("overall") == "warn" else ("PASS" if gate_summary else "WARN"),
            title="Plant replay gate defines safe fitting targets",
            conclusion=(
                "Use exact RCOU input, Bar30, IMU, DVL x/y, and gyro targets before HAN/CFD tuning. "
                "DVL z and local_position-style estimator surfaces are not safe primary targets yet."
            ),
            evidence=[
                Evidence(
                    path=str(gate_summary.get("path", "UUV-HAN/outputs/<missing contract gate>")),
                    line=None,
                    snippet=f"gate summary: {json.dumps(gate_summary, ensure_ascii=False, sort_keys=True)}",
                )
            ],
            official_refs=[OFFICIAL_REFS["mujoco_fluid"]],
        )
    )
    return checks

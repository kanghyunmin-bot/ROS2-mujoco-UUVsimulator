"""Active-runtime plant-input ownership contract source check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_runtime_plant_input_contract_check(runtime_paths: dict[str, Path]) -> Check:
    source_policy_py = runtime_paths["sitl_pwm_source_policy_py"]
    frame_handler_py = runtime_paths["sitl_pwm_frame_handler_py"]
    thruster_runtime_py = runtime_paths["thruster_actuator_runtime_py"]
    ok = (
        contains_all(
            source_policy_py,
            [
                'str(source).startswith("replay_rcout")',
                "recorded RCOUT/PWM is the authoritative plant input.",
                "_sitl_external_servo_override_until_wall",
            ],
        )
        and contains_all(
            frame_handler_py,
            [
                "replay_owns_plant = bool(self._plant_replay_mode and is_external_pwm_source(source))",
                "neutralize_disarmed_pwm_if_needed",
                "dispatch_pwm_callback(self, pwm_values)",
            ],
        )
        and contains_all(
            thruster_runtime_py,
            [
                "update_thruster_forces",
                "force_immersion_scale",
                "apply_direct_command_targets",
            ],
        )
    )
    return Check(
        check_id="active_runtime_plant_input_raw_pwm_contract",
        status="PASS" if ok else "FAIL",
        title="Plant input is raw JSON SERVO or explicit replay RCOU before thruster conversion",
        conclusion=(
            "Plant replay owns actuator input only for replay_rcout sources; otherwise raw SITL JSON SERVO PWM "
            "is passed through the plant-input handler before physical thruster force conversion."
        ),
        evidence=[
            evidence(source_policy_py, 'str(source).startswith("replay_rcout")'),
            evidence(frame_handler_py, "dispatch_pwm_callback(self, pwm_values)"),
            evidence(thruster_runtime_py, "update_thruster_forces"),
        ],
        official_refs=[OFFICIAL_REFS["ardupilot_json_sitl"]],
    )


__all__ = ["build_runtime_plant_input_contract_check"]

"""Readiness state types shared by GUI and validation tools."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RuntimeReadiness:
    mujoco_alive: bool = False
    json_sensor_transport_alive: bool = False
    json_servo_receiver_alive: bool = False
    mavlink_command_endpoint_alive: bool = False
    external_nav_alive: bool = True
    vehicle_state_known: bool = False
    arm_state_known: bool = False
    mode_state_known: bool = False
    plant_servo_rows_available: bool = False

    @property
    def command_path_ready(self) -> bool:
        return (
            self.mujoco_alive
            and self.json_sensor_transport_alive
            and self.json_servo_receiver_alive
            and self.mavlink_command_endpoint_alive
            and self.external_nav_alive
            and self.vehicle_state_known
            and self.arm_state_known
            and self.mode_state_known
        )

    @property
    def plant_input_ready(self) -> bool:
        return self.command_path_ready and self.plant_servo_rows_available

    def missing_command_gates(self) -> list[str]:
        missing: list[str] = []
        for field in (
            "mujoco_alive",
            "json_sensor_transport_alive",
            "json_servo_receiver_alive",
            "mavlink_command_endpoint_alive",
            "external_nav_alive",
            "vehicle_state_known",
            "arm_state_known",
            "mode_state_known",
        ):
            if not bool(getattr(self, field)):
                missing.append(field)
        return missing


@dataclass(frozen=True)
class CommandReadinessInputs:
    runtime: RuntimeReadiness
    arm_service_ready: bool = False
    mode_service_ready: bool = False
    rc_source_ready: bool = False
    real_start_fresh: bool = True
    real_start_required: bool = False
    real_start_ok: bool = True
    real_start_status: str = "not required"
    real_start_released: bool = True
    settle_left_s: float = 0.0
    manual_input: bool = False
    armed: bool = False
    mode: str = ""
    required_mode: str = ""
    require_runtime_command_path: bool = False
    require_fresh_rcout: bool = True


__all__ = ["CommandReadinessInputs", "RuntimeReadiness"]

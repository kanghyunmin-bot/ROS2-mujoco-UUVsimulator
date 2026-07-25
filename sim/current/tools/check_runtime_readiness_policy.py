#!/usr/bin/env python3
"""Smoke-check runtime command-readiness policy without ROS or Tk imports."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.runtime.readiness import (
    CommandReadinessInputs,
    RuntimeReadiness,
    command_readiness_label,
)


def base_inputs(runtime: RuntimeReadiness, **updates: object) -> CommandReadinessInputs:
    values = {
        "runtime": runtime,
        "arm_service_ready": True,
        "mode_service_ready": True,
        "rc_source_ready": True,
        "real_start_fresh": True,
        "real_start_required": False,
        "real_start_ok": True,
        "real_start_released": True,
        "settle_left_s": 0.0,
        "manual_input": True,
        "armed": True,
        "require_runtime_command_path": True,
        "require_fresh_rcout": True,
    }
    values.update(updates)
    return CommandReadinessInputs(**values)


def assert_label(name: str, inputs: CommandReadinessInputs, expected: str) -> None:
    actual, _style = command_readiness_label(inputs)
    if actual != expected:
        raise AssertionError(f"{name}: expected {expected!r}, got {actual!r}")


def main() -> int:
    ready = RuntimeReadiness(
        mujoco_alive=True,
        json_sensor_transport_alive=True,
        json_servo_receiver_alive=True,
        mavlink_command_endpoint_alive=True,
        vehicle_state_known=True,
        arm_state_known=True,
        mode_state_known=True,
        plant_servo_rows_available=True,
    )
    missing_mavlink = RuntimeReadiness(
        mujoco_alive=True,
        json_sensor_transport_alive=True,
        json_servo_receiver_alive=True,
        mavlink_command_endpoint_alive=False,
        vehicle_state_known=True,
        arm_state_known=True,
        mode_state_known=True,
        plant_servo_rows_available=True,
    )
    missing_sensor = RuntimeReadiness(
        mujoco_alive=True,
        json_sensor_transport_alive=False,
        json_servo_receiver_alive=True,
        mavlink_command_endpoint_alive=True,
        vehicle_state_known=True,
        arm_state_known=True,
        mode_state_known=True,
        plant_servo_rows_available=True,
    )
    missing_rcou = RuntimeReadiness(
        mujoco_alive=True,
        json_sensor_transport_alive=True,
        json_servo_receiver_alive=True,
        mavlink_command_endpoint_alive=True,
        vehicle_state_known=True,
        arm_state_known=True,
        mode_state_known=True,
        plant_servo_rows_available=False,
    )

    assert_label("ready", base_inputs(ready), "READY")
    assert_label("mavlink", base_inputs(missing_mavlink), "WAIT: SITL MAVLink")
    assert_label("sensor", base_inputs(missing_sensor), "WAIT: sensor stream")
    assert_label("rcou", base_inputs(missing_rcou), "CMD READY / RCOU WAIT")
    assert_label("manual", base_inputs(ready, manual_input=False), "WAIT: RC link")
    assert_label("armed", base_inputs(ready, armed=False), "WAIT: arm")
    assert_label(
        "external_backend_mavlink_absent",
        base_inputs(missing_mavlink, require_runtime_command_path=False),
        "READY",
    )
    print("runtime_readiness_policy=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

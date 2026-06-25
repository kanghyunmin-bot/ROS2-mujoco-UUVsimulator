"""Observation-point contract for controller parity and plant replay."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ObservationPoint:
    name: str
    source: str
    purpose: str
    resampling: str


CONTROLLER_PARITY_OBSERVATION = ObservationPoint(
    name="controller_parity_rcou",
    source="SITL MAVLink SERVO_OUTPUT_RAW telemetry",
    purpose="compare against real /mavros/rc/out",
    resampling="zero_order_hold",
)

PLANT_INPUT_OBSERVATION = ObservationPoint(
    name="plant_input_json_servo",
    source="raw ArduSub JSON servo backend",
    purpose="drive MuJoCo thrusters",
    resampling="native_sample_hold",
)

__all__ = [
    "ObservationPoint",
    "CONTROLLER_PARITY_OBSERVATION",
    "PLANT_INPUT_OBSERVATION",
]

"""Compatibility facade for plant-input validation gates."""

from __future__ import annotations

from .plant_input_gate_csv import count_csv_rows_and_activity
from .plant_input_gate_eval import evaluate_plant_input_gate
from .plant_input_gate_logs import scan_log_signatures
from .plant_input_gate_types import DISARMED_SERVO_SIGNATURES, PlantInputGateResult


__all__ = [
    "DISARMED_SERVO_SIGNATURES",
    "PlantInputGateResult",
    "count_csv_rows_and_activity",
    "scan_log_signatures",
    "evaluate_plant_input_gate",
]

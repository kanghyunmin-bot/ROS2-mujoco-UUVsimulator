"""CSV activity checks for plant-input validation gates."""

from __future__ import annotations

import csv
from pathlib import Path

from .plant_input_gate_fields import candidate_pwm_fields
from .plant_input_gate_values import finite_float, non_neutral_pwm_values


def count_csv_rows_and_activity(
    path: Path,
    *,
    neutral_center: float = 1500.0,
    neutral_tolerance: float = 2.0,
) -> tuple[int, int]:
    if not path.exists():
        return 0, 0
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = candidate_pwm_fields(reader.fieldnames or [])
        data_rows = 0
        non_neutral_rows = 0
        for row in reader:
            data_rows += 1
            values = [finite_float(row.get(field)) for field in fields]
            if non_neutral_pwm_values(
                values,
                neutral_center=neutral_center,
                neutral_tolerance=neutral_tolerance,
            ):
                non_neutral_rows += 1
    return data_rows, non_neutral_rows


__all__ = ["count_csv_rows_and_activity"]

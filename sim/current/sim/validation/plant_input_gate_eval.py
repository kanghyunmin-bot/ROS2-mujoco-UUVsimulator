"""Plant-input validation gate evaluation."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

from .plant_input_gate_csv import count_csv_rows_and_activity
from .plant_input_gate_logs import scan_log_signatures
from .plant_input_gate_types import PlantInputGateResult


def evaluate_plant_input_gate(
    *,
    csv_path: Path,
    log_paths: Iterable[Path] = (),
    require_rows: int = 1,
    require_non_neutral: bool = False,
    neutral_center: float = 1500.0,
    neutral_tolerance: float = 2.0,
    strict_log_signatures: bool = False,
) -> PlantInputGateResult:
    log_paths = list(log_paths)
    data_rows, non_neutral_rows = count_csv_rows_and_activity(
        csv_path,
        neutral_center=neutral_center,
        neutral_tolerance=neutral_tolerance,
    )
    disarmed_hits, neutral_hits = scan_log_signatures(log_paths)
    failures: list[str] = []
    if data_rows < int(require_rows):
        failures.append(f"csv_data_rows_lt_{int(require_rows)}")
    if require_non_neutral and non_neutral_rows <= 0:
        failures.append("no_non_neutral_pwm_rows")
    missing_required_activity = data_rows < int(require_rows) or (
        require_non_neutral and non_neutral_rows <= 0
    )
    if disarmed_hits > 0 and (strict_log_signatures or missing_required_activity):
        failures.append("disarmed_json_servo_signature")
    if neutral_hits > 0 and require_non_neutral and (strict_log_signatures or missing_required_activity):
        failures.append("neutral_json_servo_signature")
    return PlantInputGateResult(
        ok=not failures,
        csv_path=str(csv_path),
        data_rows=int(data_rows),
        non_neutral_rows=int(non_neutral_rows),
        log_paths=[str(path) for path in log_paths],
        disarmed_signature_hits=int(disarmed_hits),
        neutral_signature_hits=int(neutral_hits),
        failures=failures,
    )


__all__ = ["evaluate_plant_input_gate"]

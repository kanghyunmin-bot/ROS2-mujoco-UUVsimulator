"""Types and constants for plant-input validation gates."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass


DISARMED_SERVO_SIGNATURES = (
    "SITL(json) servo output ignored while disarmed",
    "SITL(json) servo stream is neutral",
)


@dataclass(frozen=True)
class PlantInputGateResult:
    ok: bool
    csv_path: str
    data_rows: int
    non_neutral_rows: int
    log_paths: list[str]
    disarmed_signature_hits: int
    neutral_signature_hits: int
    failures: list[str]

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, sort_keys=True)


__all__ = [
    "DISARMED_SERVO_SIGNATURES",
    "PlantInputGateResult",
]

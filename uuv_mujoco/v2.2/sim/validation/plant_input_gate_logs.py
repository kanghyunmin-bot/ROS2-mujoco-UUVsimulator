"""Log signature checks for plant-input validation gates."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

from .plant_input_gate_types import DISARMED_SERVO_SIGNATURES


def scan_log_signatures(paths: Iterable[Path]) -> tuple[int, int]:
    disarmed_hits = 0
    neutral_hits = 0
    for path in paths:
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        disarmed_hits += text.count(DISARMED_SERVO_SIGNATURES[0])
        neutral_hits += text.count(DISARMED_SERVO_SIGNATURES[1])
    return disarmed_hits, neutral_hits


__all__ = ["scan_log_signatures"]

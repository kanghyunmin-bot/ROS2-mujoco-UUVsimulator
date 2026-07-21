"""CSV output for ALT_HOLD diagnostics."""

from __future__ import annotations

import csv
from dataclasses import asdict
from pathlib import Path

from althold_diagnostics_contract import Snapshot


def write_csv(path: Path, rows: list[Snapshot]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(rows[0]).keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


__all__ = ["write_csv"]

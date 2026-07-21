"""CSV loading helpers for golden control-loop thruster summaries."""

from __future__ import annotations

import csv
from pathlib import Path


def load_thruster_csv(path: Path | None) -> tuple[list[dict[str, str]], list[str]]:
    if path is None or not path.exists():
        return [], []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        return rows, list(reader.fieldnames or [])


__all__ = ["load_thruster_csv"]

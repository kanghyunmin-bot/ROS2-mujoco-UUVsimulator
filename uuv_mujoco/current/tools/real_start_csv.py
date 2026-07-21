"""CSV loading and sample selection for real-start state extraction."""

from __future__ import annotations

import csv
import math
from pathlib import Path

from real_start_common import finite


def read_rows(path: Path) -> tuple[list[dict[str, str]], str]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = tuple(reader.fieldnames or ())
        time_col = "t_real_s" if "t_real_s" in fields else "t_s"
        rows = [dict(row) for row in reader if math.isfinite(finite(row.get(time_col)))]
    rows.sort(key=lambda row: finite(row.get(time_col)))
    return rows, time_col


def pick_row(rows: list[dict[str, str]], time_col: str, start_s: float) -> dict[str, str]:
    if not rows:
        raise SystemExit("real-state CSV has no timed rows")
    return min(rows, key=lambda row: abs(finite(row.get(time_col)) - start_s))

#!/usr/bin/env python3
"""Regression smoke for plant-input CSV activity detection."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.validation.plant_input_gate_csv import count_csv_rows_and_activity  # noqa: E402


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "plant.csv"
        path.write_text(
            "time_s,servo1,ch2,depth\n"
            "0.0,1500,1501,3.0\n"
            "0.1,1503,1501,3.1\n"
            "0.2,1510,bad,3.2\n",
            encoding="utf-8",
        )
        rows, active = count_csv_rows_and_activity(path, neutral_tolerance=2.0)
    assert rows == 3
    assert active == 2
    assert count_csv_rows_and_activity(Path("/tmp/does-not-exist-plant.csv")) == (0, 0)
    print("plant_input_gate_csv=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

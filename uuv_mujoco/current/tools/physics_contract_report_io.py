"""File output helpers for static physics contract audits."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

from physics_contract_types import ForceBalance


def write_static_force_balance_outputs(
    *,
    output_dir: Path,
    report: dict[str, Any],
    balances: list[ForceBalance],
) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "static_force_balance.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(ForceBalance.__dataclass_fields__.keys()))
        writer.writeheader()
        for row in balances:
            writer.writerow(row.__dict__)

    json_path = output_dir / "static_force_balance.json"
    report["csv"] = str(csv_path)
    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return csv_path, json_path


__all__ = ["write_static_force_balance_outputs"]

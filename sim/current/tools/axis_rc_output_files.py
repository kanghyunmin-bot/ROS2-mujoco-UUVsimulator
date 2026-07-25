"""CSV/JSON output writers for axis RC override checks."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

from axis_rc_contract import Phase
from axis_rc_metrics import build_health
from axis_rc_plot_render import plot_timeseries


def write_outputs(
    out_dir: Path,
    samples: list[dict[str, Any]],
    phases: list[Phase],
    summary: list[dict[str, Any]],
    metadata: dict[str, Any],
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_dict_rows(out_dir / "axis_timeseries.csv", samples)
    _write_dict_rows(out_dir / "axis_summary.csv", summary)
    (out_dir / "axis_summary.json").write_text(
        json.dumps(_summary_payload(samples, phases, summary, metadata), indent=2, ensure_ascii=False) + "\n"
    )
    plot_timeseries(out_dir / "axis_response.png", samples, phases)


def _write_dict_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = _ordered_keys(rows)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _ordered_keys(rows: list[dict[str, Any]]) -> list[str]:
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return keys


def _summary_payload(
    samples: list[dict[str, Any]],
    phases: list[Phase],
    summary: list[dict[str, Any]],
    metadata: dict[str, Any],
) -> dict[str, Any]:
    return {
        "metadata": metadata,
        "phases": [phase.__dict__ for phase in phases],
        "summary": summary,
        "sample_count": len(samples),
        "health": build_health(
            summary,
            input_mode=str(metadata.get("input_mode", "")),
            sample_hz=float(metadata.get("sample_hz", 1.0)),
            axis_s=float(metadata.get("axis_s", 1.0)),
            neutral_s=float(metadata.get("neutral_s", 1.0)),
            baseline_s=float(metadata.get("baseline_s", metadata.get("neutral_s", 1.0))),
        ),
    }


__all__ = ["write_outputs"]

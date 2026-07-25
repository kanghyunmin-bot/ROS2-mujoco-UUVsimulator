"""Report writers for source-level contract audits."""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

from audit_code_contract_markdown_sections import evidence_lines, metadata_lines, summary_lines
from audit_code_contract_types import Check


def build_payload(checks: list[Check], metadata: dict[str, object]) -> dict[str, object]:
    return {
        "metadata": metadata,
        "checks": [asdict(check) for check in checks],
        "counts": {
            "pass": sum(1 for check in checks if check.status == "PASS"),
            "warn": sum(1 for check in checks if check.status == "WARN"),
            "fail": sum(1 for check in checks if check.status == "FAIL"),
        },
    }


def write_json(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n", encoding="utf-8")


def write_markdown(path: Path, checks: list[Check], metadata: dict[str, object]) -> None:
    lines: list[str] = []
    lines.append("# Code-Level Contract Source Audit")
    lines.append("")
    lines.append("This report compares official-facing contracts with the local ArduSub 4.1.2 and active MuJoCo runtime source paths.")
    lines.append("")
    lines.extend(metadata_lines(metadata))
    lines.append("")
    lines.extend(summary_lines(checks))
    lines.append("")
    lines.extend(evidence_lines(checks))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")

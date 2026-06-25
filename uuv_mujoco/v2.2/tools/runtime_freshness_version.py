"""Refresh active runtime provenance metadata."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from runtime_freshness_version_payload import build_runtime_version


def refresh_runtime_version(
    inputs: dict[str, Any], report: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    path = Path(inputs["runtime_version_path"])
    result: dict[str, Any] = {"attempted": True, "changed": False, "path": str(path), "skipped": ""}
    hard_failures = [issue for issue in report["issues"] if issue["level"] == "fail"]
    if hard_failures:
        result["skipped"] = "freshness check has hard failures"
        return inputs.get("runtime_version", {}), result

    existing = inputs.get("runtime_version", {})
    updated = build_runtime_version(inputs, existing, freshness_status=str(report.get("status") or "unknown"))
    if existing == updated:
        return updated, result

    path.write_text(json.dumps(updated, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    result["changed"] = True
    return updated, result


__all__ = ["build_runtime_version", "refresh_runtime_version"]

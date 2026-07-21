"""Contract-gate summary loading for source-level audits."""

from __future__ import annotations

import json
from pathlib import Path


def contract_gate_summary_candidates(repo_root: Path) -> list[Path]:
    return [
        repo_root / "UUV-HAN/outputs/validate_proposed_added_mass_full90_20260601/contract_gate.json",
        repo_root / "UUV-HAN/outputs/added_mass_residual_full90_20260601/baseline_contract_gate.json",
    ]


def load_contract_gate_summary_from(repo_root: Path, rel_path) -> dict[str, object]:
    for path in contract_gate_summary_candidates(repo_root):
        if not path.exists():
            continue
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        return {
            "path": rel_path(path),
            "overall": payload.get("overall"),
            "han_cfd_ready": payload.get("han_cfd_ready"),
            "warnings": payload.get("warnings", []),
            "passes": payload.get("passes", []),
        }
    return {}


__all__ = ["contract_gate_summary_candidates", "load_contract_gate_summary_from"]

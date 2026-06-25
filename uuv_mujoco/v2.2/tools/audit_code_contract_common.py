"""Common path and evidence helpers for source-level contract audits."""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Iterable

from audit_code_contract_gate_summary import load_contract_gate_summary_from
from audit_code_contract_types import Evidence


REPO_ROOT = Path(__file__).resolve().parents[3]
COMPAT_V22_ROOT = Path(__file__).resolve().parents[1]
ACTIVE_RUNTIME_ALIAS = REPO_ROOT / "uuv_mujoco" / "current"
ACTIVE_RUNTIME_ROOT = ACTIVE_RUNTIME_ALIAS if ACTIVE_RUNTIME_ALIAS.exists() else COMPAT_V22_ROOT
ARDUPILOT_ROOT = REPO_ROOT / "ardupilot"


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def rel(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def line_of(text: str, needle: str) -> int | None:
    idx = text.find(needle)
    if idx < 0:
        return None
    return text[:idx].count("\n") + 1


def evidence(path: Path, needle: str, label: str | None = None) -> Evidence:
    text = read_text(path)
    return Evidence(path=rel(path), line=line_of(text, needle), snippet=label or needle)


def contains_all(path: Path, needles: Iterable[str]) -> bool:
    text = read_text(path)
    return all(needle in text for needle in needles)


def git_output(args: list[str], cwd: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", *args],
            cwd=str(cwd),
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return ""


def load_contract_gate_summary() -> dict[str, object]:
    return load_contract_gate_summary_from(REPO_ROOT, rel)

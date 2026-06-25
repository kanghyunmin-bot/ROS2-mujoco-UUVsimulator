"""Output helpers for ArduPilot integrity preflight."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any


def write_json_report(path: Path | None, payload: dict[str, Any]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def print_preflight_result(issues: list[str]) -> None:
    if issues:
        for issue in issues:
            print(f"[ardupilot-preflight] {issue}", file=sys.stderr)
    else:
        print("[ardupilot-preflight] ArduPilot watched files are clean")


def preflight_exit_code(*, failed: bool, allow_dirty: bool) -> int:
    if failed and not allow_dirty:
        print(
            "[ardupilot-preflight] refusing authoritative closed-loop validation. "
            "Set UUV_ALLOW_DIRTY_ARDUPILOT=1 only for smoke tests.",
            file=sys.stderr,
        )
        return 1
    return 0


__all__ = ["preflight_exit_code", "print_preflight_result", "write_json_report"]

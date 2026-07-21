"""DataFlash BIN path selection helpers."""

from __future__ import annotations

from pathlib import Path


def latest_bin(default_root: Path) -> Path:
    logs = sorted(default_root.glob("*.BIN"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not logs:
        raise SystemExit(f"no BIN logs found in {default_root}")
    return logs[0]


__all__ = ["latest_bin"]

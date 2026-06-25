"""Path filtering helpers for the refactor inventory tool."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable


def normalized_rel(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def should_skip(path: Path, root: Path, excludes: set[str]) -> bool:
    rel = normalized_rel(path, root)
    parts = set(path.relative_to(root).parts)
    if parts & {".git", "__pycache__"}:
        return True
    return any(rel == item or rel.startswith(f"{item}/") for item in excludes)


def iter_python_files(root: Path, excludes: set[str]) -> Iterable[Path]:
    for path in sorted(root.rglob("*.py")):
        if should_skip(path, root, excludes):
            continue
        yield path


__all__ = ["iter_python_files", "normalized_rel", "should_skip"]

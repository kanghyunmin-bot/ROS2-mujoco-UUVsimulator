"""Bounded process-log allocation and incremental marker scanning."""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Iterable, TextIO


DEFAULT_LOGS_PER_PREFIX = 20


def allocate_process_log_path(
    log_dir: Path,
    prefix: str,
    *,
    keep: int | None = None,
) -> Path:
    """Return a collision-free path after reserving one bounded log slot."""

    log_dir.mkdir(parents=True, exist_ok=True)
    keep_count = _configured_keep_count() if keep is None else max(1, int(keep))
    _prune_process_logs(log_dir, prefix, keep=max(0, keep_count - 1))
    stem = f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}"
    candidate = log_dir / f"{stem}.log"
    suffix = 1
    while candidate.exists():
        candidate = log_dir / f"{stem}_{suffix}.log"
        suffix += 1
    return candidate


def open_process_log(
    log_dir: Path,
    prefix: str,
    *,
    keep: int | None = None,
) -> tuple[Path, TextIO]:
    path = allocate_process_log_path(log_dir, prefix, keep=keep)
    return path, path.open("w", encoding="utf-8", buffering=1)


class IncrementalLogMatcher:
    """Read only text appended since the previous marker check."""

    def __init__(self, path: Path, markers: Iterable[str]) -> None:
        self.path = Path(path)
        self.markers = tuple(str(marker) for marker in markers if str(marker))
        self.offset = 0
        self.characters_read = 0
        self._carry = ""
        self._carry_limit = max((len(marker) for marker in self.markers), default=1) - 1

    def poll(self) -> bool:
        try:
            size = self.path.stat().st_size
            if size < self.offset:
                self.offset = 0
                self._carry = ""
            with self.path.open("r", encoding="utf-8", errors="replace") as stream:
                stream.seek(self.offset)
                chunk = stream.read()
                self.offset = stream.tell()
        except OSError:
            return False
        self.characters_read += len(chunk)
        text = self._carry + chunk
        if any(marker in text for marker in self.markers):
            return True
        self._carry = text[-self._carry_limit :] if self._carry_limit > 0 else ""
        return False


def _configured_keep_count() -> int:
    raw = os.environ.get("UUV_GUI_LOG_KEEP_PER_PREFIX", str(DEFAULT_LOGS_PER_PREFIX))
    try:
        value = int(str(raw).strip())
    except (TypeError, ValueError):
        value = DEFAULT_LOGS_PER_PREFIX
    return max(1, min(200, value))


def _prune_process_logs(log_dir: Path, prefix: str, *, keep: int) -> None:
    candidates: list[tuple[int, str, Path]] = []
    for path in log_dir.glob(f"{prefix}_*.log"):
        try:
            if path.is_file():
                candidates.append((path.stat().st_mtime_ns, path.name, path))
        except OSError:
            continue
    candidates.sort(reverse=True)
    for _mtime_ns, _name, stale in candidates[max(0, int(keep)) :]:
        try:
            stale.unlink(missing_ok=True)
        except OSError:
            # Retention is housekeeping and must never block a launch.
            continue


__all__ = [
    "DEFAULT_LOGS_PER_PREFIX",
    "IncrementalLogMatcher",
    "allocate_process_log_path",
    "open_process_log",
]

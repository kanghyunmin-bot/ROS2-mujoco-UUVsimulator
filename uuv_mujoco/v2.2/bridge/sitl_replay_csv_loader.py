"""Shared CSV loading policy for SITL replay records."""

from __future__ import annotations

import csv
from collections.abc import Callable
from pathlib import Path
from typing import Optional, TypeVar

from bridge.sitl_replay_common import replay_log

T = TypeVar("T")


def load_replay_csv_records(
    path_raw: str,
    *,
    missing_message: str,
    failure_message: str,
    row_parser: Callable[[dict[str, str]], T | None],
    sort_key: Callable[[T], float],
    log: Optional[Callable[[str], None]] = None,
) -> list[T]:
    if not path_raw:
        return []
    path = Path(path_raw).expanduser()
    if not path.exists():
        replay_log(log, f"{missing_message}: {path}")
        return []
    records: list[T] = []
    try:
        with path.open("r", newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                record = row_parser(row)
                if record is not None:
                    records.append(record)
    except Exception as exc:
        replay_log(log, f"{failure_message} {path}: {exc}")
        return []
    records.sort(key=sort_key)
    return records


__all__ = ["load_replay_csv_records"]

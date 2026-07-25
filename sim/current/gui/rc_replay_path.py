"""Path helpers for GUI RC override replay loading."""

from __future__ import annotations

from pathlib import Path


def rc_replay_bag_uri(path_text: str) -> str:
    path = Path(path_text).expanduser()
    if path.suffix == ".db3":
        return str(path.parent)
    return str(path)


__all__ = ["rc_replay_bag_uri"]

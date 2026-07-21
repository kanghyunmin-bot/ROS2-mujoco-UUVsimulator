"""CSV file handling for MuJoCo thruster debug output."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import TextIO

from sim.physics.thruster_debug import build_thruster_debug_header


def open_thruster_debug_file(
    *,
    path_text: str,
    thruster_names: list[str],
    log: Callable[[str], None],
) -> TextIO | None:
    path_text = path_text.strip()
    if not path_text:
        return None

    try:
        debug_path = Path(path_text).expanduser()
        debug_path.parent.mkdir(parents=True, exist_ok=True)
        debug_file = debug_path.open("w", encoding="utf-8", buffering=1)
        debug_file.write(",".join(build_thruster_debug_header(thruster_names)) + "\n")
        log(f"[debug] thruster CSV: {debug_path}")
        return debug_file
    except OSError as exc:
        log(f"[debug] failed to open thruster CSV {path_text}: {exc}")
        return None


__all__ = ["open_thruster_debug_file"]

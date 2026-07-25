"""Atomic GUI file writes and bounded backups outside active configuration.

Files under the active simulator tree are backed up below
``generated/backups``.  Files supplied by tests or external callers use a
local ``.backups`` directory so a temporary fixture remains self-contained.
"""

from __future__ import annotations

from datetime import datetime
import os
from pathlib import Path
import shutil
import stat
import tempfile


SIM_STACK_DIR = Path(__file__).resolve().parents[1]
GENERATED_BACKUP_ROOT = SIM_STACK_DIR / "generated" / "backups"
DEFAULT_BACKUP_RETENTION = 5


def backup_file(
    source: Path,
    *,
    category: str,
    retention: int = DEFAULT_BACKUP_RETENTION,
) -> Path:
    """Create one complete backup and retain only the newest copies."""

    source = Path(source)
    if not source.is_file():
        raise FileNotFoundError(source)
    backup_dir = _backup_directory(source, category)
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    backup_path = backup_dir / f"{source.name}.{stamp}.bak"

    with tempfile.NamedTemporaryFile(
        dir=backup_dir,
        prefix=f".{source.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        temp_path = Path(handle.name)
    try:
        shutil.copy2(source, temp_path)
        temp_path.replace(backup_path)
    finally:
        temp_path.unlink(missing_ok=True)

    _prune_backups(backup_dir, source.name, retention=max(1, int(retention)))
    return backup_path


def atomic_write_text(path: Path, content: str, *, encoding: str = "utf-8") -> None:
    """Replace *path* atomically after flushing a complete temporary file."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    target_mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o644
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding=encoding,
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        temp_path = Path(handle.name)
        handle.write(content)
        handle.flush()
        os.fsync(handle.fileno())
    os.chmod(temp_path, target_mode)
    try:
        temp_path.replace(path)
    finally:
        temp_path.unlink(missing_ok=True)


def _backup_directory(source: Path, category: str) -> Path:
    clean_category = "".join(
        character if character.isalnum() or character in {"-", "_"} else "_"
        for character in str(category).strip()
    ) or "misc"
    try:
        source.resolve().relative_to(SIM_STACK_DIR.resolve())
    except ValueError:
        return source.parent / ".backups" / clean_category
    return GENERATED_BACKUP_ROOT / clean_category


def _prune_backups(backup_dir: Path, source_name: str, *, retention: int) -> None:
    backups = sorted(
        backup_dir.glob(f"{source_name}.*.bak"),
        key=lambda path: (path.stat().st_mtime_ns, path.name),
        reverse=True,
    )
    for stale in backups[retention:]:
        stale.unlink(missing_ok=True)


__all__ = [
    "DEFAULT_BACKUP_RETENTION",
    "GENERATED_BACKUP_ROOT",
    "atomic_write_text",
    "backup_file",
]

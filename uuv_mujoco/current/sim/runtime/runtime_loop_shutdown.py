"""Shutdown helpers for top-level runtime loop entrypoints."""

from __future__ import annotations

from collections.abc import Callable, Sequence


ShutdownCallback = Callable[[], None]


def run_shutdown_callbacks(callbacks: Sequence[ShutdownCallback]) -> None:
    """Best-effort cleanup for resources owned by the main runner."""

    for callback in callbacks:
        try:
            callback()
        except Exception as exc:  # pragma: no cover - defensive cleanup path.
            print(f"[runtime] cleanup callback failed: {exc}", flush=True)


__all__ = ["ShutdownCallback", "run_shutdown_callbacks"]

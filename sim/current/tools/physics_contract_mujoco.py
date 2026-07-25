"""Lazy MuJoCo import for physics contract tools."""

from __future__ import annotations

from typing import Any


def load_mujoco() -> Any:
    try:
        import mujoco  # type: ignore[import-not-found]
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "MuJoCo is required for physics contract audits. Run this tool with "
            "the MuJoCo-enabled runtime Python instead of the system Python."
        ) from exc
    return mujoco


class LazyMujoco:
    def __getattr__(self, name: str) -> Any:
        return getattr(load_mujoco(), name)


mujoco = LazyMujoco()


__all__ = ["load_mujoco", "mujoco"]

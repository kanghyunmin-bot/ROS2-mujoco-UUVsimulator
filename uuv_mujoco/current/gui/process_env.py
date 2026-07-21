"""Environment helpers shared by GUI process controls."""

from __future__ import annotations

import os


def env_flag(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def arg_present(args: list[str], option: str) -> bool:
    return any(arg == option or arg.startswith(f"{option}=") for arg in args)


def default_sim_stack_backend() -> str:
    return os.environ.get("UUV_SITL_BACKEND_DEFAULT", "native").strip().lower() or "native"


def sim_stack_backend() -> str:
    backend = os.environ.get("UUV_SITL_BACKEND", default_sim_stack_backend())
    backend = backend.strip().lower()
    return backend or default_sim_stack_backend()


__all__ = ["arg_present", "default_sim_stack_backend", "env_flag", "sim_stack_backend"]

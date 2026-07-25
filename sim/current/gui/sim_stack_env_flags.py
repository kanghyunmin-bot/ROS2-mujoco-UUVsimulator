"""Small normalization helpers for GUI simulator stack env and args."""

from __future__ import annotations

from typing import Collection, Mapping, Sequence


TRUE_VALUES = {"1", "true", "yes", "on", "enable", "enabled"}
STRICT_TRUE_VALUES = {"1", "true", "yes", "on"}


def env_bool(
    env: Mapping[str, str],
    name: str,
    default: str = "0",
    *,
    true_values: Collection[str] = TRUE_VALUES,
) -> bool:
    return str(env.get(name, default)).strip().lower() in true_values


def env_flag_default(env: Mapping[str, str], name: str, default: bool = False) -> bool:
    raw = env.get(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() in STRICT_TRUE_VALUES


def arg_present(args: Sequence[str], option: str) -> bool:
    return any(arg == option or arg.startswith(f"{option}=") for arg in args)


def any_arg_present(args: Sequence[str], options: tuple[str, ...]) -> bool:
    return any(arg_present(args, option) for option in options)

"""Wrapper-only argument filtering for GUI simulator launchers."""

from __future__ import annotations

from typing import Sequence


WRAPPER_ONLY_ARGS = {
    "--direct-mavlink",
    "--with-qgc-stop",
    "--no-wait-ready",
    "--sitl-no-rebuild",
    "--sitl-rebuild",
    "--sitl-no-display",
    "--no-ekf-stable",
    "--param-tune",
    "--wipe-eeprom",
    "--keep-eeprom",
    "--no-reset",
}


def filter_wrapper_only_args(extra_args: Sequence[str] | None) -> tuple[list[str], list[str]]:
    args: list[str] = []
    dropped: list[str] = []
    for arg in extra_args or []:
        option = arg.split("=", 1)[0]
        if option in WRAPPER_ONLY_ARGS:
            dropped.append(arg)
            continue
        args.append(arg)
    return args, dropped


__all__ = ["WRAPPER_ONLY_ARGS", "filter_wrapper_only_args"]

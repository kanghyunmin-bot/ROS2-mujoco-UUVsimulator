"""Small helpers for assigning Tk variables on the GUI app object."""

from __future__ import annotations

import tkinter as tk


def bool_vars(owner, defaults: dict[str, bool]) -> None:
    for name, value in defaults.items():
        setattr(owner, name, tk.BooleanVar(value=value))


def double_vars(owner, defaults: dict[str, float]) -> None:
    for name, value in defaults.items():
        setattr(owner, name, tk.DoubleVar(value=value))


def string_vars(owner, defaults: dict[str, str]) -> None:
    for name, value in defaults.items():
        setattr(owner, name, tk.StringVar(value=value))


__all__ = ["bool_vars", "double_vars", "string_vars", "tk"]

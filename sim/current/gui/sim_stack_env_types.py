"""Typed return values for GUI simulator stack environment helpers."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class InitialDepthArgs:
    args: tuple[str, ...] = ()
    events: tuple[str, ...] = ()


@dataclass(frozen=True)
class NormalizedSimExtraArgs:
    args: tuple[str, ...] = ()
    events: tuple[str, ...] = ()

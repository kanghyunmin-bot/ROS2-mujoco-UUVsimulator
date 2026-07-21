"""Mutable runtime state owned by the Ping360 simulator lifecycle."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ping360_history import Ping360History
from .ping360_settings import build_effective_settings
from .ping360_sweep import Ping360SweepState
from .ping360_types import Ping360Config, Ping360EffectiveSettings


@dataclass
class Ping360RuntimeState:
    geomgroup: np.ndarray
    rng: np.random.Generator
    sweep: Ping360SweepState
    settings: Ping360EffectiveSettings
    history: Ping360History


def create_ping360_runtime_state(config: Ping360Config) -> Ping360RuntimeState:
    settings = build_effective_settings(config)
    geomgroup = np.ones(6, dtype=np.uint8)
    geomgroup[5] = 0
    return Ping360RuntimeState(
        geomgroup=geomgroup,
        rng=np.random.default_rng(360),
        sweep=Ping360SweepState(),
        settings=settings,
        history=Ping360History(settings.number_of_samples),
    )


def refresh_ping360_runtime_state(runtime: Ping360RuntimeState, config: Ping360Config) -> bool:
    runtime.settings = build_effective_settings(config)
    return runtime.history.resize(runtime.settings.number_of_samples)


__all__ = [
    "Ping360RuntimeState",
    "create_ping360_runtime_state",
    "refresh_ping360_runtime_state",
]

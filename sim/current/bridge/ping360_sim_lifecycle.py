"""Lifecycle helpers for the public Ping360Simulator facade."""

from __future__ import annotations

import mujoco

from .ping360_model_ids import lookup_ping360_model_ids
from .ping360_runtime_state import create_ping360_runtime_state, refresh_ping360_runtime_state
from .ping360_types import Ping360Config


def initialize_ping360_simulator(owner: object, model: mujoco.MjModel, config: Ping360Config) -> None:
    owner.model = model
    owner.config = config
    owner.site_id, owner.base_body_id = lookup_ping360_model_ids(model, config.site_name)
    owner._runtime = create_ping360_runtime_state(config)
    owner.settings = owner._runtime.settings
    owner._latest = None
    owner._next_profile_t = -1.0
    owner._ping_number = 0


def ping360_simulator_active(owner: object) -> bool:
    return bool(owner.config.enabled and owner.site_id >= 0)


def update_ping360_simulator_config(owner: object, config: Ping360Config) -> None:
    owner.config = config
    owner.site_id, owner.base_body_id = lookup_ping360_model_ids(owner.model, config.site_name)
    refresh_ping360_simulator_runtime(owner)


def refresh_ping360_simulator_runtime(owner: object) -> None:
    if refresh_ping360_runtime_state(owner._runtime, owner.config):
        owner._latest = None
    owner.settings = owner._runtime.settings


__all__ = [
    "initialize_ping360_simulator",
    "ping360_simulator_active",
    "refresh_ping360_simulator_runtime",
    "update_ping360_simulator_config",
]

"""Real-start actuator state seeding for plant replay release."""

from __future__ import annotations

import json
import os
from typing import Any, Iterable

import numpy as np


def _bounded_scale(raw: object, *, default: float) -> float:
    try:
        scale = float(raw)
    except (TypeError, ValueError):
        scale = float(default)
    return max(0.0, min(1.0, scale))


def _seed_scale_from_env(name: str, *, default: float) -> float:
    return _bounded_scale(os.environ.get(name), default=default)


def _seed_state_from_env() -> dict[str, float]:
    raw = os.environ.get("UUV_REAL_START_THRUSTER_STATE_SEED_JSON", "").strip()
    if not raw:
        return {}
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        print(f"[runtime] ignoring invalid release actuator seed JSON: {exc}", flush=True)
        return {}
    if not isinstance(payload, dict):
        print("[runtime] ignoring release actuator seed JSON: expected object", flush=True)
        return {}
    out: dict[str, float] = {}
    for name, value in payload.items():
        try:
            seed = float(value)
        except (TypeError, ValueError):
            continue
        if np.isfinite(seed):
            out[str(name)] = float(np.clip(seed, -1.0, 1.0))
    return out


def seed_thruster_state_from_current_targets(
    *,
    thruster_actuator_runtime: Any,
    base_id: int,
    vertical_names: Iterable[str] = (),
    horizontal_names: Iterable[str] = (),
    horizontal_order: Iterable[str] = (),
    horizontal_allocator: Any = None,
) -> None:
    """Seed first-order thruster state from the latched PWM target.

    Plant replay starts from an in-flight real state. Without this seed, the
    rigid-body qvel is real-started but every thruster first-order state starts
    from zero, which creates an artificial release transient.
    """
    names = list(getattr(thruster_actuator_runtime, "all_thruster_names", []))
    target = getattr(thruster_actuator_runtime, "target", {})
    state = getattr(thruster_actuator_runtime, "state", {})
    explicit_seed = _seed_state_from_env()
    global_scale = _seed_scale_from_env("UUV_REAL_START_THRUSTER_STATE_SEED_SCALE", default=1.0)
    vertical_scale = _seed_scale_from_env(
        "UUV_REAL_START_THRUSTER_STATE_SEED_VERTICAL_SCALE",
        default=global_scale,
    )
    horizontal_scale = _seed_scale_from_env(
        "UUV_REAL_START_THRUSTER_STATE_SEED_HORIZONTAL_SCALE",
        default=global_scale,
    )
    forward_scale = _seed_scale_from_env(
        "UUV_REAL_START_THRUSTER_STATE_SEED_FORWARD_SCALE",
        default=horizontal_scale,
    )
    sway_scale = _seed_scale_from_env(
        "UUV_REAL_START_THRUSTER_STATE_SEED_SWAY_SCALE",
        default=horizontal_scale,
    )
    yaw_axis_scale = _seed_scale_from_env(
        "UUV_REAL_START_THRUSTER_STATE_SEED_YAW_SCALE",
        default=horizontal_scale,
    )
    vertical_set = set(vertical_names)
    horizontal_set = set(horizontal_names)
    horizontal_seed = _horizontal_axis_scaled_seed(
        target=target,
        horizontal_order=list(horizontal_order),
        horizontal_allocator=horizontal_allocator,
        forward_scale=forward_scale,
        sway_scale=sway_scale,
        yaw_axis_scale=yaw_axis_scale,
    )
    for name in names:
        if name in explicit_seed:
            state[name] = float(explicit_seed[name])
        elif name in horizontal_seed:
            state[name] = float(horizontal_seed[name])
        else:
            seed_scale = global_scale
            if name in vertical_set:
                seed_scale = vertical_scale
            elif name in horizontal_set:
                seed_scale = horizontal_scale
            state[name] = seed_scale * float(target.get(name, 0.0))
    thruster_actuator_runtime.update_forces(0.0, base_id=base_id)
    mode = "history_json" if explicit_seed else "current_target"
    print(
        f"[runtime] release actuator state seeded mode={mode} "
        f"scale={global_scale:.3f} vertical_scale={vertical_scale:.3f} "
        f"horizontal_scale={horizontal_scale:.3f} forward_scale={forward_scale:.3f} "
        f"sway_scale={sway_scale:.3f} yaw_scale={yaw_axis_scale:.3f} "
        f"explicit_thrusters={len(explicit_seed)}",
        flush=True,
    )


def _horizontal_axis_scaled_seed(
    *,
    target: Any,
    horizontal_order: list[str],
    horizontal_allocator: Any,
    forward_scale: float,
    sway_scale: float,
    yaw_axis_scale: float,
) -> dict[str, float]:
    if horizontal_allocator is None or not horizontal_order:
        return {}
    pseudo_inverse = getattr(horizontal_allocator, "pseudo_inverse", None)
    if pseudo_inverse is None:
        return {}
    try:
        mixer = np.asarray(pseudo_inverse, dtype=np.float64)
        target_vec = np.asarray(
            [float(target.get(name, 0.0)) for name in horizontal_order],
            dtype=np.float64,
        )
        axis_cmd, *_ = np.linalg.lstsq(mixer, target_vec, rcond=None)
        scaled_axis_cmd = np.asarray(
            [
                float(forward_scale) * float(axis_cmd[0]),
                float(sway_scale) * float(axis_cmd[1]),
                float(yaw_axis_scale) * float(axis_cmd[2]),
            ],
            dtype=np.float64,
        )
        seed_vec = np.clip(mixer @ scaled_axis_cmd, -1.0, 1.0)
    except Exception:
        return {}
    return {name: float(value) for name, value in zip(horizontal_order, seed_vec)}


__all__ = ["seed_thruster_state_from_current_targets"]

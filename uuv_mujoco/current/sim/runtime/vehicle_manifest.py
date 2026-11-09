"""Load the explicit motor channel and direction mapping for a custom vehicle."""

from __future__ import annotations

import json
from pathlib import Path

from physics.thruster_mapping import vehicle_thruster_mapping


def parse_thrusters(manifest_dict: dict) -> tuple[tuple[str, ...], tuple[str, ...], tuple[str, ...], tuple[int, ...]]:
    """Return horizontal names, vertical names, PWM channel names, and PWM signs."""
    thrusters = manifest_dict.get("thrusters")
    if not isinstance(thrusters, list) or not thrusters:
        raise ValueError("vehicle manifest must contain a nonempty thrusters array")
    horizontal, vertical, names, signs = [], [], [], []
    for item in thrusters:
        if not isinstance(item, dict):
            raise ValueError("each thruster must be an object")
        name, kind = item.get("name"), item.get("kind")
        if not isinstance(name, str) or not name.strip() or name in names:
            raise ValueError(f"invalid or duplicate thruster name: {name!r}")
        if kind not in ("horizontal", "vertical"):
            raise ValueError(f"invalid kind for thruster {name!r}: {kind!r}")
        for field in ("pwm_sign", "forward_sign", "spin_sign"):
            value = item.get(field, 1)
            if isinstance(value, bool) or value not in (-1, 1):
                raise ValueError(f"{name}.{field} must be +1 or -1")
        names.append(name)
        (horizontal if kind == "horizontal" else vertical).append(name)
        signs.append(int(item.get("pwm_sign", 1)))
    return tuple(horizontal), tuple(vertical), tuple(names), tuple(signs)


def load_vehicle_manifest(path: str | Path) -> dict:
    """Read and validate a custom vehicle manifest."""
    with Path(path).open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    if not isinstance(manifest, dict):
        raise ValueError("vehicle manifest must be a JSON object")
    parse_thrusters(manifest)
    return manifest


def runtime_thruster_mapping(args):
    """Return the built-in or custom mapping and reject unknown SITL mixers."""
    vehicle = getattr(args, "vehicle", "ut27")
    path = getattr(args, "vehicle_manifest", None)
    if vehicle == "custom":
        if not path:
            raise ValueError("--vehicle custom requires --vehicle-manifest")
        if getattr(args, "sitl", False):
            raise ValueError("--sitl is unsupported for custom vehicles: mixer is unknown")
        if getattr(args, "pwm_layout", "standard") != "standard":
            raise ValueError("custom vehicles use the PWM mapping in their manifest")
        return parse_thrusters(load_vehicle_manifest(path))
    if path:
        raise ValueError("--vehicle-manifest requires --vehicle custom")
    return vehicle_thruster_mapping(vehicle, pwm_layout=getattr(args, "pwm_layout", "standard"))

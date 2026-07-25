"""Fluid-model selection helpers for MuJoCo runtime."""

from __future__ import annotations

FLUID_MODEL_ALIASES = {
    "legacy": "legacy",
    "custom": "legacy",
    "python": "legacy",
    "python-6dof": "legacy",
    "current": "current",
    "ellipsoid": "current",
    "builtin-ellipsoid": "current",
}


def normalize_fluid_model(raw: str) -> str:
    fluid_model = FLUID_MODEL_ALIASES.get(str(raw).strip().lower())
    if fluid_model is None:
        print(f"[physics] unknown fluid model: {raw}", flush=True)
        print("Available fluid models:", flush=True)
        for name in ("current", "ellipsoid", "builtin-ellipsoid", "legacy", "custom"):
            print(f"  - {name}", flush=True)
        raise SystemExit(2)
    return fluid_model

"""Fluid-model selection helpers for MuJoCo runtime."""

from __future__ import annotations

from typing import Any, Mapping

FLUID_MODEL_ALIASES = {
    "legacy": "legacy",
    "custom": "legacy",
    "python": "legacy",
    "python-6dof": "legacy",
    "distributed": "legacy",
    "patch": "legacy",
    "current": "current",
    "ellipsoid": "current",
    "builtin-ellipsoid": "current",
}
_DISTRIBUTED_REQUEST_ALIASES = frozenset({"distributed", "patch"})


def normalize_fluid_model(raw: str) -> str:
    fluid_model = FLUID_MODEL_ALIASES.get(str(raw).strip().lower())
    if fluid_model is None:
        print(f"[physics] unknown fluid model: {raw}", flush=True)
        print("Available fluid models:", flush=True)
        for name in (
            "current",
            "ellipsoid",
            "builtin-ellipsoid",
            "distributed",
            "legacy",
            "custom",
        ):
            print(f"  - {name}", flush=True)
        raise SystemExit(2)
    return fluid_model


def validate_requested_fluid_model_profile(
    raw: str,
    sim_profile: Mapping[str, Any],
) -> None:
    """Reject a distributed CLI request without an active patch profile."""

    requested = str(raw).strip().lower()
    if requested not in _DISTRIBUTED_REQUEST_ALIASES:
        return
    payload = sim_profile.get("distributed_hydrodynamics")
    if not isinstance(payload, Mapping) or payload.get("active") is not True:
        raise ValueError(
            f"--fluid-model {requested} requires a profile with "
            "distributed_hydrodynamics.active=true; recommended profile: "
            "research_pool_distributed"
        )


__all__ = [
    "FLUID_MODEL_ALIASES",
    "normalize_fluid_model",
    "validate_requested_fluid_model_profile",
]

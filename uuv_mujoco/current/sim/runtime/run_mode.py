"""Runtime mode contract for closed-loop and plant replay runs."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RuntimeMode:
    name: str
    plant_replay_direct_rcout: bool


def resolve_runtime_mode(env_get, env_flag) -> RuntimeMode:
    """Resolve runtime mode from environment without touching simulator state."""

    mode = str(env_get("UUV_RUN_MODE", "closed_loop")).strip().lower()
    if mode not in {"closed_loop", "plant_replay"}:
        print(f"[mode] invalid UUV_RUN_MODE={mode!r}; falling back to closed_loop", flush=True)
        mode = "closed_loop"
    plant_replay_direct_rcout = bool(
        mode == "plant_replay"
        and env_flag("ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE", False)
    )
    print(f"[mode] UUV_RUN_MODE={mode}", flush=True)
    return RuntimeMode(
        name=mode,
        plant_replay_direct_rcout=plant_replay_direct_rcout,
    )


__all__ = ["RuntimeMode", "resolve_runtime_mode"]

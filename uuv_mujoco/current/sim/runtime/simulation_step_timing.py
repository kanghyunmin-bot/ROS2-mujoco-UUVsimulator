"""Low-overhead phase timing for simulation-step diagnostics."""

from __future__ import annotations

import os
import time
from typing import Any


def record_step_phase(runtime: Any, phase: str, elapsed_s: float) -> None:
    if os.environ.get("UUV_MUJOCO_STEP_STATS", "1").strip().lower() not in {"1", "true", "yes", "on"}:
        return
    state = getattr(runtime, "_step_timing_state", None)
    now = time.perf_counter()
    if state is None:
        state = {"started": now, "steps": 0, "phases": {}}
        runtime._step_timing_state = state
    phases = state["phases"]
    phases[phase] = float(phases.get(phase, 0.0)) + max(0.0, float(elapsed_s))
    if phase == "total":
        state["steps"] += 1
    window_s = now - float(state["started"])
    if window_s < 5.0 or phase != "total":
        return
    ordered = sorted(phases.items(), key=lambda item: item[1], reverse=True)
    details = " ".join(f"{name}={1000.0 * value / max(state['steps'], 1):.2f}ms" for name, value in ordered)
    print(f"[runtime] step stats: hz={state['steps'] / window_s:.1f} {details}", flush=True)
    runtime._step_timing_state = {"started": now, "steps": 0, "phases": {}}


__all__ = ["record_step_phase"]

"""Runtime configuration parsing for dynamic MuJoCo fluidcoef updates."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np

from sim.physics.dynamic_fluidcoef_runtime_knobs import (
    configure_dynamic_fluidcoef_transient_knobs,
    configure_dynamic_fluidcoef_update_knobs,
)
from sim.physics.dynamic_fluidcoef_runtime_state import (
    attach_dynamic_fluidcoef_setup_state,
    initialize_dynamic_fluidcoef_runtime_buffers,
)
from sim.physics.dynamic_fluidcoef_types import DynamicFluidcoefSetup


def configure_dynamic_fluidcoef_runtime(
    runtime: Any,
    *,
    setup: DynamicFluidcoefSetup,
    water_current_world: np.ndarray,
    fluid_geom_names: dict[int, str],
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
    to_float_array: Callable[[object], np.ndarray | None],
) -> None:
    """Populate dynamic-fluidcoef runtime state from setup arrays and env knobs."""

    attach_dynamic_fluidcoef_setup_state(
        runtime,
        setup=setup,
        water_current_world=water_current_world,
        fluid_geom_names=fluid_geom_names,
    )
    configure_dynamic_fluidcoef_update_knobs(runtime, env_float=env_float)
    configure_dynamic_fluidcoef_transient_knobs(
        runtime,
        env_float=env_float,
        to_float_array=to_float_array,
    )
    initialize_dynamic_fluidcoef_runtime_buffers(runtime, env_flag=env_flag)


__all__ = ["configure_dynamic_fluidcoef_runtime"]

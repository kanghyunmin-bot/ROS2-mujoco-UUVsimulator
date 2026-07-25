"""Compatibility facade for dynamic MuJoCo fluid coefficient runtime knobs."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from sim.physics.dynamic_fluidcoef_transient_mask import (
    configure_dynamic_fluidcoef_transient_mask,
)
from sim.physics.dynamic_fluidcoef_transient_mode import (
    configure_dynamic_fluidcoef_transient_mode,
)
from sim.physics.dynamic_fluidcoef_transient_thresholds import (
    configure_dynamic_fluidcoef_transient_thresholds,
)
from sim.physics.dynamic_fluidcoef_update_knobs import (
    configure_dynamic_fluidcoef_update_knobs,
)


def configure_dynamic_fluidcoef_transient_knobs(
    runtime,
    *,
    env_float: Callable[[str, float], float],
    to_float_array: Callable[[object], np.ndarray | None],
) -> None:
    configure_dynamic_fluidcoef_transient_mode(runtime)
    configure_dynamic_fluidcoef_transient_thresholds(runtime, env_float=env_float)
    configure_dynamic_fluidcoef_transient_mask(
        runtime,
        env_float=env_float,
        to_float_array=to_float_array,
    )


__all__ = [
    "configure_dynamic_fluidcoef_transient_knobs",
    "configure_dynamic_fluidcoef_update_knobs",
]

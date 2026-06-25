"""Transient mode selection for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

import os


DISABLED_TRANSIENT_MODES = {
    "",
    "0",
    "false",
    "none",
    "off",
    "disabled",
}


def configure_dynamic_fluidcoef_transient_mode(runtime) -> None:
    runtime.transient_mode = str(
        os.environ.get(
            "UUV_DYNAMIC_FLUIDCOEF_TRANSIENT_MODE",
            runtime.cfg.get("transient_mode", "log_decay"),
        )
    ).strip().lower()
    runtime.transient_enabled = runtime.transient_mode not in DISABLED_TRANSIENT_MODES


__all__ = ["DISABLED_TRANSIENT_MODES", "configure_dynamic_fluidcoef_transient_mode"]

"""Compatibility exports for MuJoCo UUV GUI configuration."""

from __future__ import annotations

from .config_backend import *  # noqa: F401,F403
from .config_backend import pilot_control_mode as _pilot_control_mode
from .config_env import *  # noqa: F401,F403
from .config_env import env_float_default as _env_float_default
from .config_env import env_int as _env_int
from .config_env import profile_default_update_ms as _profile_default_update_ms
from .config_env import runtime_profile as _runtime_profile
from .config_paths import *  # noqa: F401,F403
from .config_physics import *  # noqa: F401,F403
from .config_rc import *  # noqa: F401,F403
from .config_ui import *  # noqa: F401,F403

__all__ = [
    name
    for name in globals()
    if not name.startswith("__") and name not in {"annotations"}
]

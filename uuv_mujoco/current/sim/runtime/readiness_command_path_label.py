"""Runtime command-path readiness label policy."""

from __future__ import annotations

from .readiness_label_types import NOT_READY_STYLE, ReadinessLabel
from .readiness_types import CommandReadinessInputs


def command_path_readiness_label(inputs: CommandReadinessInputs) -> ReadinessLabel | None:
    runtime = inputs.runtime
    if not inputs.require_runtime_command_path or runtime.command_path_ready:
        return None

    missing = runtime.missing_command_gates()
    if "mavlink_command_endpoint_alive" in missing:
        return "WAIT: SITL MAVLink", NOT_READY_STYLE
    if "external_nav_alive" in missing:
        return "WAIT: ExternalNav", NOT_READY_STYLE
    if "json_sensor_transport_alive" in missing:
        return "WAIT: sensor stream", NOT_READY_STYLE
    return "WAIT: command path", NOT_READY_STYLE


__all__ = ["command_path_readiness_label"]

"""Command-readiness label policy shared by GUI and validation tools."""

from __future__ import annotations

from .readiness_command_path_label import command_path_readiness_label
from .readiness_control_label import control_readiness_label
from .readiness_label_types import ReadinessLabel
from .readiness_preflight_label import preflight_readiness_label
from .readiness_types import CommandReadinessInputs


def command_readiness_label(inputs: CommandReadinessInputs) -> ReadinessLabel:
    """Return the GUI command-readiness label and ttk style.

    This is dependency-free on purpose.  The GUI can call it live, while tests
    and contract tools can verify the same readiness policy without importing
    ROS or Tk.
    """

    for stage in (preflight_readiness_label, command_path_readiness_label):
        label = stage(inputs)
        if label is not None:
            return label
    return control_readiness_label(inputs)


__all__ = ["command_readiness_label"]

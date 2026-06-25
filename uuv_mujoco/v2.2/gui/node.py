"""ROS node used by the MuJoCo UUV control GUI."""

from __future__ import annotations

from .node_bindings import bind_uuv_gui_node_methods
from .node_init import initialize_uuv_gui_node
from .runtime import Node


class UuvGuiNode(Node):
    def __init__(self, namespace: str, backend: str):
        super().__init__("uuv_control_gui")
        initialize_uuv_gui_node(self, namespace, backend)

    def _topic(self, suffix: str) -> str:
        if not self._base_ns:
            return f"/{suffix.lstrip('/')}"
        return f"{self._base_ns}/{suffix.lstrip('/')}"

    def vehicle_info_supported(self) -> bool:
        return bool(self._vehicle_info_supported)

    def probe_backend(self) -> None:
        self._probe_backend()

    def push_event(self, text: str) -> None:
        self._push_event(text)


bind_uuv_gui_node_methods(UuvGuiNode)

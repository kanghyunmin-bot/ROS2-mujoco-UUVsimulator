"""Trigger service helper for GUI command paths."""

from __future__ import annotations

from .node_trigger_service_callbacks import handle_trigger_future, trigger_service_ready
from .runtime import Trigger


def _call_trigger_service(self, client, label: str, on_success=None, on_done=None) -> bool:
    if not trigger_service_ready(client):
        self._push_event(f"{label}: service unavailable")
        return False
    future = client.call_async(Trigger.Request())
    future.add_done_callback(
        lambda fut: handle_trigger_future(self, fut, label, on_success=on_success, on_done=on_done)
    )
    return True


__all__ = ["_call_trigger_service"]

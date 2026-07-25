"""Trigger service helper for axis RC checks."""

from __future__ import annotations

import time
from typing import Any

from std_srvs.srv import Trigger

from axis_rc_service_spin import publish_neutral_and_spin, wait_for_future


def call_trigger_service(node: Any, client: Any, service_name: str, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + float(timeout)
    _wait_for_trigger_service(node, client, service_name, deadline)
    future = client.call_async(Trigger.Request())
    wait_for_future(node, future, deadline)
    _raise_if_trigger_failed(future, service_name)


def _wait_for_trigger_service(node: Any, client: Any, service_name: str, deadline: float) -> None:
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.1):
            return
        publish_neutral_and_spin(node, timeout_sec=0.05)
    raise RuntimeError(f"{service_name} service did not become ready")


def _raise_if_trigger_failed(future: Any, service_name: str) -> None:
    if not future.done() or future.result() is None:
        raise RuntimeError(f"{service_name} call timed out")
    result = future.result()
    if not bool(result.success):
        raise RuntimeError(f"{service_name} failed: {result.message}")


__all__ = ["call_trigger_service"]

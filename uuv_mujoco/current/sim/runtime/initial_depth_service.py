"""ROS service installation for initial-depth hold release."""

from __future__ import annotations

from typing import Any, Callable


INITIAL_DEPTH_RELEASE_SERVICE = "/mujoco/release_initial_depth_hold"


def install_initial_depth_hold_service(
    *,
    ros_bridge,
    hold_state,
    request_release_fn: Callable[[str], bool],
) -> list[Any]:
    """Install the ROS service that releases the initial-depth hold."""
    services: list[Any] = []
    if not _service_can_be_installed(ros_bridge, hold_state):
        return services

    try:
        from std_srvs.srv import Trigger

        services.append(_create_release_service(ros_bridge, Trigger, request_release_fn))
        print(
            "[runtime] initial depth hold service enabled: "
            f"{INITIAL_DEPTH_RELEASE_SERVICE}",
            flush=True,
        )
    except Exception as exc:
        print(f"[ros2] initial depth hold service unavailable: {exc}", flush=True)
    return services


def _service_can_be_installed(ros_bridge: Any, hold_state: Any) -> bool:
    _ = hold_state
    return bool(
        ros_bridge is not None
        and getattr(ros_bridge, "node", None) is not None
    )


def _create_release_service(ros_bridge: Any, trigger_type: Any, request_release_fn: Callable[[str], bool]) -> Any:
    def _release_initial_depth_hold(_request, response):
        queued = bool(request_release_fn("service"))
        response.success = True
        response.message = (
            "initial depth hold release queued"
            if queued
            else "initial depth hold already inactive"
        )
        return response

    return ros_bridge.node.create_service(
        trigger_type,
        INITIAL_DEPTH_RELEASE_SERVICE,
        _release_initial_depth_hold,
    )


__all__ = ["INITIAL_DEPTH_RELEASE_SERVICE", "install_initial_depth_hold_service"]

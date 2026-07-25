"""Initial-depth release service wiring for the runtime control bridge."""

from __future__ import annotations

from .initial_depth_runtime import install_initial_depth_hold_service


def install_runtime_initial_depth_services(
    *,
    ros_bridge_runtime,
    initial_depth_hold: dict,
    initial_depth_runtime,
):
    def release_initial_depth_hold(reason: str) -> bool:
        return initial_depth_runtime.release(reason, ros_bridge=ros_bridge_runtime.get())

    def request_initial_depth_hold_release(reason: str) -> bool:
        return initial_depth_runtime.request_release(reason)

    def process_pending_initial_depth_release() -> bool:
        return initial_depth_runtime.process_pending_release(ros_bridge=ros_bridge_runtime.get())

    services = install_initial_depth_hold_service(
        ros_bridge=ros_bridge_runtime.get(),
        hold_state=initial_depth_hold,
        request_release_fn=request_initial_depth_hold_release,
    )
    return release_initial_depth_hold, process_pending_initial_depth_release, services


__all__ = ["install_runtime_initial_depth_services"]

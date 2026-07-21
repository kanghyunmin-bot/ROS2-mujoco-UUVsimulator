"""Failure handling for the mutable ROS bridge runtime wrapper."""

from __future__ import annotations


def shutdown_ros_bridge_once(runtime) -> None:
    if runtime.bridge is None:
        return
    bridge = runtime.bridge
    runtime.bridge = None
    bridge.shutdown()


def disable_ros_bridge_after_failure(runtime, *, label: str, exc: Exception) -> None:
    print(f"[ros2] {label} failed, disabling bridge: {exc}", flush=True)
    try:
        shutdown_ros_bridge_once(runtime)
    except Exception as shutdown_exc:
        print(f"[ros2] bridge shutdown after {label} failure also failed: {shutdown_exc}", flush=True)


__all__ = ["disable_ros_bridge_after_failure", "shutdown_ros_bridge_once"]

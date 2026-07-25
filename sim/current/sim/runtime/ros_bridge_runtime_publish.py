"""Publish/spin helpers for the mutable ROS bridge runtime wrapper."""

from __future__ import annotations

from sim.runtime.ros_bridge_runtime_failure import disable_ros_bridge_after_failure


def publish_ros_bridge_once(
    runtime,
    *,
    data,
    initial_depth_hold,
    real_start_status,
) -> None:
    if runtime.bridge is None:
        return
    try:
        if hasattr(runtime.bridge, "set_sitl_initial_depth_hold_active"):
            runtime.bridge.set_sitl_initial_depth_hold_active(bool(initial_depth_hold["active"]))
        runtime.bridge.publish(data)
        real_start_status.publish()
    except Exception as exc:
        disable_ros_bridge_after_failure(runtime, label="publish", exc=exc)


def publish_qgc_video_once(runtime, *, qgc_video, data) -> None:
    qgc_video.publish(data, runtime.bridge)


def spin_ros_bridge_once(runtime) -> None:
    if runtime.bridge is None:
        return
    try:
        runtime.bridge.spin_once()
    except Exception as exc:
        disable_ros_bridge_after_failure(runtime, label="spin_once", exc=exc)


__all__ = ["publish_qgc_video_once", "publish_ros_bridge_once", "spin_ros_bridge_once"]

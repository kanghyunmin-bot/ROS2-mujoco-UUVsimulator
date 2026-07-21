"""Initial-depth hold release sequence."""

from __future__ import annotations

from typing import Any


def release_initial_depth_hold_runtime(runtime: Any, reason: str, *, ros_bridge=None) -> bool:
    if not runtime.state["active"]:
        return False
    runtime.state.mark_released(float(runtime.data.time))
    runtime.reset_release_state()
    runtime.apply_release_velocity_state()
    runtime.seed_release_actuator_state()
    runtime.mujoco.mj_forward(runtime.model, runtime.data)
    publish_release_sensor_snapshot(ros_bridge, runtime.data)
    print(f"[runtime] initial depth hold released ({reason})", flush=True)
    return True


def publish_release_sensor_snapshot(ros_bridge: Any, data: Any) -> None:
    if ros_bridge is None:
        return
    try:
        _mark_hold_inactive(ros_bridge)
        _force_next_publish(ros_bridge)
        ros_bridge.publish(data)
        print("[runtime] release sensor snapshot published", flush=True)
    except Exception as exc:
        print(f"[ros2] release sensor snapshot publish failed: {exc}", flush=True)


def _mark_hold_inactive(ros_bridge: Any) -> None:
    if hasattr(ros_bridge, "set_sitl_initial_depth_hold_active"):
        ros_bridge.set_sitl_initial_depth_hold_active(False)


def _force_next_publish(ros_bridge: Any) -> None:
    if hasattr(ros_bridge, "force_next_publish"):
        ros_bridge.force_next_publish()


__all__ = ["publish_release_sensor_snapshot", "release_initial_depth_hold_runtime"]

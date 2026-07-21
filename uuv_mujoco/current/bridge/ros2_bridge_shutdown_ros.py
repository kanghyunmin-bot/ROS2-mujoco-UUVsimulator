"""ROS executor, node, and context shutdown helpers."""

from __future__ import annotations

from .ros2_bridge_shutdown_guard import run_shutdown_step


def remove_executor_node(self) -> None:
    run_shutdown_step(_remove_executor_node, self)


def shutdown_executor(self) -> None:
    run_shutdown_step(_shutdown_executor, self)


def destroy_ros_node(self) -> None:
    run_shutdown_step(_destroy_ros_node, self)


def shutdown_ros_context(self) -> None:
    run_shutdown_step(_shutdown_ros_context, self)


def _remove_executor_node(self) -> None:
    if self._executor is not None and self.node is not None:
        self._executor.remove_node(self.node)


def _shutdown_executor(self) -> None:
    if self._executor is not None:
        self._executor.shutdown(timeout_sec=0.0)


def _destroy_ros_node(self) -> None:
    if self.node is not None:
        self.node.destroy_node()


def _shutdown_ros_context(self) -> None:
    if self._ros_context is not None and self._ros_context.ok():
        self._ros_context.shutdown()


__all__ = [
    "destroy_ros_node",
    "remove_executor_node",
    "shutdown_executor",
    "shutdown_ros_context",
]

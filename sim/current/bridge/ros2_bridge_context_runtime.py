"""ROS2 context, node, executor, and endpoint setup."""

from __future__ import annotations

from .ros2_endpoints import create_ros2_endpoints


def initialize_ros2_context_and_node(self, imports: dict[str, object]) -> None:
    context_factory = imports["Context"]
    node_factory = imports["Node"]
    signal_handler_options = imports["SignalHandlerOptions"]

    self._ros_context = context_factory()
    init_kwargs = {"args": None, "context": self._ros_context}
    if signal_handler_options is not None:
        init_kwargs["signal_handler_options"] = signal_handler_options.NO
    self.rclpy.init(**init_kwargs)
    self.node = node_factory("uuv_mujoco_bridge", context=self._ros_context)
    self._executor = self.SingleThreadedExecutor(context=self._ros_context)
    self._executor.add_node(self.node)


def initialize_ros2_endpoints(self) -> None:
    tf_qos = self.QoSProfile(depth=50)
    latched_qos = self.QoSProfile(
        depth=1,
        history=self.HistoryPolicy.KEEP_LAST,
        reliability=self.ReliabilityPolicy.RELIABLE,
        durability=self.DurabilityPolicy.TRANSIENT_LOCAL,
    )
    create_ros2_endpoints(
        self,
        q10=10,
        q1=1,
        tf_qos=tf_qos,
        latched_qos=latched_qos,
    )


__all__ = ["initialize_ros2_context_and_node", "initialize_ros2_endpoints"]

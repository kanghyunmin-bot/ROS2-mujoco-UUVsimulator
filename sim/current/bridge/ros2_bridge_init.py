"""ROS2 runtime initialization for the MuJoCo bridge."""

from __future__ import annotations

from .ros2_bridge_context_runtime import initialize_ros2_context_and_node, initialize_ros2_endpoints
from .ros2_bridge_imports import load_ros2_runtime_imports
from .ros2_bridge_message_bindings import bind_ros2_runtime_imports
from .ros2_bridge_runtime import PublisherDemandCache
from .ros2_bridge_startup_log import log_ros2_bridge_startup
from .ros2_bridge_static_context import initialize_static_context_publisher
from .sitl_env import env_to_float


def configure_ros_runtime_state(self) -> None:
    """Initialize ROS runtime placeholders before optional rclpy setup."""

    self.node = None
    self.rclpy = None
    self._ros_context = None
    self._executor = None
    self.RCOut = None
    self.DVLMsg = None
    self.DVLDRMsg = None
    self.CollectorState = None
    self.Clock = None
    self._publisher_demand = PublisherDemandCache(
        default_probe_period_s=env_to_float("ROS2_UUV_DEMAND_PROBE_PERIOD_S", 1.0)
    )
    self._static_context_publisher = None
    self._static_tf_published = False
    self._robot_description_text = self._load_robot_description_text()
    self._robot_description_pub_period_s = 1.0
    self._robot_description_next_t = 0.0


def init_ros_runtime(self) -> None:
    imports = load_ros2_runtime_imports()
    bind_ros2_runtime_imports(self, imports)
    if self._real_pkg_compat and self.DVLMsg is None:
        raise RuntimeError(
            "strict real-package compatibility requires dvl_msgs/msg/DVL; "
            "build and source the dvl_msgs workspace before launching"
        )
    initialize_ros2_context_and_node(self, imports)
    initialize_ros2_endpoints(self)
    initialize_static_context_publisher(self)
    self._ros_ok = True
    if self._ros_executor_spin_thread_enabled:
        self._start_ros_spin_thread()
    log_ros2_bridge_startup(self)


__all__ = ["configure_ros_runtime_state", "init_ros_runtime"]

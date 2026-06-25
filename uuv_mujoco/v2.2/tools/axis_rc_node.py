"""ROS2 node for axis RC override validation."""

from __future__ import annotations

import time
from typing import Any

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ManualControl, OverrideRCIn, RCIn, RCOut, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from axis_rc_node_callbacks import AxisRcNodeCallbacks
from axis_rc_node_control import AxisRcNodeControl
from axis_rc_node_services import AxisRcNodeServices


class AxisRcOverrideCheck(AxisRcNodeCallbacks, AxisRcNodeControl, AxisRcNodeServices, Node):
    def __init__(
        self,
        sample_hz: float,
        input_mode: str = "rc-override",
        invert_heave_rc: bool = False,
    ) -> None:
        super().__init__("axis_rc_override_check")
        self.sample_dt = 1.0 / max(1.0, float(sample_hz))
        self.input_mode = str(input_mode)
        self.invert_heave_rc = bool(invert_heave_rc)
        self._rc_released = False
        self.start_wall = time.monotonic()
        self._last_sample_wall = -1.0
        self.current_phase = "init"
        self.recording = False
        self.current_axis: str | None = None
        self.current_command = 0.0
        self.last_command_publish_wall = -1.0
        self.last_command_publish_t = float("nan")
        self.last_command_sequence = 0
        self.last_command_mode = ""

        self.state: State | None = None
        self.imu: Imu | None = None
        self.dvl_twist: TwistStamped | None = None
        self.depth: Float32 | None = None
        self.local_odom: Odometry | None = None
        self.rc_in: RCIn | None = None
        self.rc_out: RCOut | None = None

        self.samples: list[dict[str, Any]] = []
        self.phases: list[Any] = []

        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 1)
        self.manual_pub = self.create_publisher(ManualControl, "/mavros/manual_control/send", 1)
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.switch_initial_depth_hold_client = self.create_client(
            Trigger,
            "/mujoco/switch_initial_depth_hold_to_target",
        )
        self.release_initial_depth_hold_client = self.create_client(
            Trigger,
            "/mujoco/release_initial_depth_hold",
        )

        self.create_subscription(State, "/mavros/state", self._on_state, 20)
        self.create_subscription(RCIn, "/mavros/rc/in", self._on_rc_in, 1)
        self.create_subscription(RCOut, "/mavros/rc/out", self._on_rc_out, 1)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 50)
        self.create_subscription(TwistStamped, "/dvl/velocity", self._on_dvl_twist, 50)
        self.create_subscription(Float32, "/depth", self._on_depth, 20)
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._on_local_odom, 20)
        self.create_timer(self.sample_dt, self._sample)

    def elapsed(self) -> float:
        return time.monotonic() - self.start_wall


__all__ = ["AxisRcOverrideCheck"]

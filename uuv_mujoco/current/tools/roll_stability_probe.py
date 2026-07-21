"""ROS2 probe node used by roll stability sweeps."""

from __future__ import annotations

import time
from typing import Any

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn, RCOut, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from roll_stability_probe_callbacks import RollStabilityCallbacksMixin
from roll_stability_probe_commands import RollStabilityCommandMixin
from roll_stability_probe_rc import RollStabilityRcMixin
from roll_stability_probe_sequence import RollStabilitySequenceMixin


class StabilityProbe(
    RollStabilitySequenceMixin,
    RollStabilityCommandMixin,
    RollStabilityRcMixin,
    RollStabilityCallbacksMixin,
    Node,
):
    def __init__(self) -> None:
        super().__init__("uuv_roll_stability_probe")
        self.state: State | None = None
        self.samples: list[dict[str, Any]] = []
        self.depth_samples: list[tuple[float, float]] = []
        self.rc_samples: list[tuple[float, list[int]]] = []
        self.latest_gyro = (float("nan"), float("nan"), float("nan"))
        self.latest_depth = float("nan")
        self.sample_enabled = False
        self.t0 = time.monotonic()

        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.state_sub = self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/mujoco/ground_truth/pose", self._on_pose, 50)
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self._on_imu, 50)
        self.depth_sub = self.create_subscription(Float32, "/depth", self._on_depth, 10)
        self.rc_out_sub = self.create_subscription(RCOut, "/mavros/rc/out", self._on_rc_out, 30)
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")


__all__ = ["StabilityProbe"]

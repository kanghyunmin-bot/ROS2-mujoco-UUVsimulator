import json
import math
from copy import deepcopy

from nav_msgs.msg import Odometry
import rclpy

from kmu26_auv_web_gui.ros_interface import LocalizationRosNode
from kmu26_auv_web_gui.ros_interface import PATH_MAX_POINTS
from kmu26_auv_web_gui.ros_interface import PATH_RECENT_POINTS


def test_status_path_is_bounded_while_preserving_shape_and_recent_points() -> None:
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init(args=[])
    node = LocalizationRosNode()
    try:
        input_points = PATH_MAX_POINTS * 6
        for index in range(input_points):
            node._append_path_point(index * 0.02, math.sin(index * 0.004))

        snapshot = node.snapshot()
        path = snapshot["path"]
        assert len(path) <= PATH_MAX_POINTS
        assert snapshot["path_count"] == len(path)
        assert snapshot["path_diagnostics"]["max_points"] == PATH_MAX_POINTS
        assert path[0] == {"x": 0.0, "y": 0.0}
        assert path[-1]["x"] == (input_points - 1) * 0.02

        # The latest section remains at full input resolution while older
        # samples are shape-preserving, evenly spaced representatives.
        recent = path[-PATH_RECENT_POINTS:]
        assert all(
            math.isclose(
                recent[index + 1]["x"] - recent[index]["x"],
                0.02,
                abs_tol=1.0e-12,
            )
            for index in range(len(recent) - 1)
        )
        assert len(json.dumps(path, separators=(",", ":"))) < 100_000
    finally:
        node.destroy_node()
        if owned_context and rclpy.ok():
            rclpy.shutdown()


def test_localization_jump_only_resets_visual_path_not_pose_or_control() -> None:
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init(args=[])
    node = LocalizationRosNode()
    try:
        node._append_path_point(1.0, 2.0)
        node._web_control["enabled"] = True
        node._web_control["active"] = True
        node._web_control["axes"] = {
            "forward": 0.25,
            "lateral": -0.1,
            "vertical": 0.05,
            "yaw": 0.2,
        }
        control_before = deepcopy(node._web_control)

        jump = Odometry()
        jump.header.frame_id = "odom"
        jump.child_frame_id = "base_link"
        jump.pose.pose.position.x = 1600.0
        jump.pose.pose.position.y = -1760.0
        jump.pose.pose.position.z = -1.25
        jump.pose.pose.orientation.w = 1.0
        node._on_odom(jump)

        snapshot = node.snapshot()
        assert snapshot["pose"] == {
            "x": 1600.0,
            "y": -1760.0,
            "z": -1.25,
            "yaw": 0.0,
        }
        assert snapshot["path"] == [{"x": 1600.0, "y": -1760.0}]
        assert snapshot["path_diagnostics"]["visual_reset_count"] == 1
        assert node._web_control == control_before

        broken = Odometry()
        broken.pose.pose.position.x = math.nan
        broken.pose.pose.position.y = -1760.0
        broken.pose.pose.orientation.w = 1.0
        node._on_odom(broken)
        assert math.isnan(node._pose["x"])
        assert node._path == [{"x": 1600.0, "y": -1760.0}]
        assert node._web_control == control_before

        recovered = Odometry()
        recovered.pose.pose.position.x = 3.0
        recovered.pose.pose.position.y = 4.0
        recovered.pose.pose.orientation.w = 1.0
        node._on_odom(recovered)
        assert node._pose["x"] == 3.0
        assert node._pose["y"] == 4.0
        assert node._path == [{"x": 3.0, "y": 4.0}]
        assert node._path_reset_count == 2
        assert node._web_control == control_before
    finally:
        node.destroy_node()
        if owned_context and rclpy.ok():
            rclpy.shutdown()

"""Aggregate lazy ROS message builders for publish jobs."""

from __future__ import annotations

from .ros2_publish_builder_core import build_core_publish_builders
from .ros2_publish_builder_dvl import build_dvl_publish_builders
from .ros2_publish_builder_mavros import build_mavros_publish_builders
from .ros2_publish_builder_odometry import build_odometry_publish_builders
from .ros2_publish_builder_ping360 import build_ping360_publish_builders
from .ros2_publish_builder_status import build_status_publish_builders
from .ros2_publish_course_buoys import build_course_buoy_publish_builders
from .ros2_publish_state import RosPublishState
from .ros2_hydrophone_sim import build_hydrophone_publish_builders
from .ros2_stereo_image import build_stereo_publish_builders


def build_ros_publish_builders(self, data, stamp, state: RosPublishState) -> dict[str, object]:
    builders: dict[str, object] = {}
    builders.update(build_core_publish_builders(self, stamp, state))
    builders.update(build_status_publish_builders(self, stamp, state))
    builders.update(build_dvl_publish_builders(self, stamp, state))
    builders.update(build_mavros_publish_builders(self, stamp, state))
    builders.update(build_odometry_publish_builders(self, stamp, state))
    builders.update(build_ping360_publish_builders(self, data, stamp, state))
    builders.update(build_hydrophone_publish_builders(self, data, stamp, state))
    builders.update(build_stereo_publish_builders(self, data, stamp))
    builders.update(build_course_buoy_publish_builders(self, data, stamp, state))
    return builders

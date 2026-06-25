"""Static TF and robot description publisher setup for the bridge."""

from __future__ import annotations

from .ros2_bridge_runtime import StaticContextPublisher
from .ros2_tf_messages import build_static_tf_specs, build_tf_message


def initialize_static_context_publisher(self) -> None:
    self._static_tf_specs = build_static_tf_specs(
        model=self.model,
        imu_site_id=self._imu_site_id,
        bar30_site_id=self._bar30_site_id,
        dvl_site_id=self._dvl_site_id,
        ping360_site_id=self._ping360_site_id,
        ping360_frame_id=self._ping360_config.frame_id,
        cam_left_site_id=self._cam_left_site_id,
        cam_right_site_id=self._cam_right_site_id,
    )
    self._static_context_publisher = StaticContextPublisher(
        tf_static_pub=self.pub_tf_static,
        robot_description_pub=self.pub_robot_description,
        string_factory=self.String,
        build_tf_message=lambda stamp, specs: build_tf_message(
            self.TFMessage,
            self.TransformStamped,
            stamp,
            specs,
        ),
        safe_publish=self._safe_publish,
        static_tf_specs=self._static_tf_specs,
        robot_description_text=self._robot_description_text,
        robot_description_pub_period_s=self._robot_description_pub_period_s,
    )


__all__ = ["initialize_static_context_publisher"]

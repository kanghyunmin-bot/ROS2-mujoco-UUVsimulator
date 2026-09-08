#!/usr/bin/env python3
# Copyright (c) 2026, KMU Underwater Robot Team.
# SPDX-License-Identifier: MIT

"""Publish RViz markers for the physical A50 contract and filtered path."""

from __future__ import annotations

import math
from collections import deque
from typing import Any

import rclpy
from auv_dvl_a50_msg.msg import DVL
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray


BEAM_AZIMUTHS_DEG = (45.0, 135.0, 225.0, 315.0)
BEAM_TILT_DEG = 22.5


def beam_direction_frd(beam_id: int) -> tuple[float, float, float]:
    """Return a default simulator A50 beam direction in the DVL FRD frame."""
    if beam_id < 0 or beam_id >= len(BEAM_AZIMUTHS_DEG):
        raise ValueError(f"unsupported A50 beam id: {beam_id}")
    tilt = math.radians(BEAM_TILT_DEG)
    azimuth = math.radians(BEAM_AZIMUTHS_DEG[beam_id])
    return (
        math.sin(tilt) * math.cos(azimuth),
        math.sin(tilt) * math.sin(azimuth),
        math.cos(tilt),
    )


def normalized_beam_direction_frd(
    values: Any, beam_id: int
) -> tuple[float, float, float]:
    """Validate and normalize one configurable A50 beam direction."""
    direction = tuple(float(value) for value in values)
    if len(direction) != 3 or not all(math.isfinite(value) for value in direction):
        raise ValueError(f"beam {beam_id} direction must contain three finite values")
    norm = math.sqrt(sum(value * value for value in direction))
    if norm <= 1.0e-9:
        raise ValueError(f"beam {beam_id} direction must be non-zero")
    return tuple(value / norm for value in direction)


def beam_has_valid_range(beam: Any) -> bool:
    """Return whether a beam carries a valid physical slant range [m]."""
    distance = float(beam.distance)
    return bool(beam.valid) and math.isfinite(distance) and 0.05 <= distance <= 50.0


def dvl_sample_passes_input_gate(message: Any) -> bool:
    """Apply the default DVL-to-EKF input gates for status coloring."""
    velocity = message.velocity
    if not bool(message.velocity_valid):
        return False
    if not all(
        math.isfinite(float(value))
        for value in (velocity.x, velocity.y, velocity.z)
    ):
        return False
    if not math.isfinite(float(message.altitude)) or float(message.altitude) < 0.05:
        return False
    if not math.isfinite(float(message.fom)) or float(message.fom) > 0.05:
        return False
    if sum(bool(beam.valid) for beam in message.beams) < 4:
        return False
    covariance = tuple(float(value) for value in message.covariance)
    if len(covariance) == 9:
        diagonal = (covariance[0], covariance[4], covariance[8])
    elif len(covariance) == 36:
        diagonal = (covariance[0], covariance[7], covariance[14])
    else:
        return False
    return all(math.isfinite(value) and value <= 1.0 for value in diagonal)


def path_state_must_reset(
    last_stamp_s: float | None,
    last_frame_id: str | None,
    stamp_s: float,
    frame_id: str,
) -> bool:
    """Return whether a time reset or frame change invalidates the path."""
    time_went_back = last_stamp_s is not None and stamp_s < last_stamp_s - 1.0e-9
    frame_changed = last_frame_id is not None and frame_id != last_frame_id
    return time_went_back or frame_changed


def _point(x: float, y: float, z: float) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


class DvlLocalizationVisualizer(Node):
    """Adapt raw A50 data and filtered odometry into focused RViz surfaces."""

    def __init__(self) -> None:
        super().__init__("dvl_localization_visualizer")
        self.dvl_topic = str(self.declare_parameter("dvl_topic", "/dvl/data").value)
        self.odom_topic = str(
            self.declare_parameter("odom_topic", "/odometry/filtered").value
        )
        self.marker_topic = str(
            self.declare_parameter("marker_topic", "/dvl/markers").value
        )
        self.path_topic = str(
            self.declare_parameter(
                "path_topic", "/localization/filtered_path"
            ).value
        )
        self.velocity_scale_s = max(
            0.1, float(self.declare_parameter("velocity_scale_s", 5.0).value)
        )
        self.marker_lifetime_s = max(
            0.1, float(self.declare_parameter("marker_lifetime_s", 0.35).value)
        )
        self.max_path_points = max(
            100, int(self.declare_parameter("max_path_points", 2500).value)
        )
        self.path_min_distance_m = max(
            0.0, float(self.declare_parameter("path_min_distance_m", 0.01).value)
        )
        self.path_min_period_s = max(
            0.02, float(self.declare_parameter("path_min_period_s", 0.10).value)
        )
        self.beam_directions_frd = tuple(
            normalized_beam_direction_frd(
                self.declare_parameter(
                    f"beam_direction_frd_{beam_id}",
                    list(beam_direction_frd(beam_id)),
                ).value,
                beam_id,
            )
            for beam_id in range(len(BEAM_AZIMUTHS_DEG))
        )

        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, reliable
        )
        self.path_publisher = self.create_publisher(Path, self.path_topic, reliable)
        self.create_subscription(
            DVL, self.dvl_topic, self._on_dvl, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odometry, qos_profile_sensor_data
        )
        self._path: deque[PoseStamped] = deque(maxlen=self.max_path_points)
        self._last_path_stamp_s: float | None = None
        self._last_path_position: tuple[float, float, float] | None = None
        self._path_frame_id: str | None = None
        self.get_logger().info(
            f"DVL {self.dvl_topic} -> {self.marker_topic}; "
            f"EKF {self.odom_topic} -> {self.path_topic}"
        )

    def _marker(self, message: DVL, namespace: str, marker_id: int) -> Marker:
        marker = Marker()
        marker.header = message.header
        if not marker.header.frame_id:
            marker.header.frame_id = "dvl_link"
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        seconds = int(self.marker_lifetime_s)
        marker.lifetime = Duration(
            sec=seconds,
            nanosec=int((self.marker_lifetime_s - seconds) * 1.0e9),
        )
        return marker

    def _on_dvl(self, message: DVL) -> None:
        markers = MarkerArray()
        clear = self._marker(message, "dvl", 0)
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        by_id = {
            int(beam.id): beam
            for beam in message.beams
            if 0 <= int(beam.id) < len(BEAM_AZIMUTHS_DEG)
        }
        valid_count = 0
        for beam_id in range(len(BEAM_AZIMUTHS_DEG)):
            direction = self.beam_directions_frd[beam_id]
            guide = self._marker(message, "dvl_beam_guides", beam_id)
            guide.type = Marker.LINE_LIST
            guide.scale.x = 0.008
            guide.color.r = 0.38
            guide.color.g = 0.45
            guide.color.b = 0.50
            guide.color.a = 0.35
            guide.points = [
                _point(0.0, 0.0, 0.0),
                _point(*(0.32 * value for value in direction)),
            ]
            markers.markers.append(guide)

            beam = by_id.get(beam_id)
            if beam is None or not beam_has_valid_range(beam):
                continue
            valid_count += 1
            distance = float(beam.distance)
            endpoint = tuple(distance * value for value in direction)

            ray = self._marker(message, "dvl_beam_ranges", beam_id)
            ray.type = Marker.ARROW
            ray.scale.x = 0.014
            ray.scale.y = 0.045
            ray.scale.z = 0.060
            ray.color.r = 0.04
            ray.color.g = 0.82
            ray.color.b = 0.96
            ray.color.a = 0.90
            ray.points = [_point(0.0, 0.0, 0.0), _point(*endpoint)]
            markers.markers.append(ray)

            hit = self._marker(message, "dvl_bottom_hits", beam_id)
            hit.type = Marker.SPHERE
            hit.pose.position = _point(*endpoint)
            hit.scale.x = 0.055
            hit.scale.y = 0.055
            hit.scale.z = 0.055
            hit.color.r = 0.18
            hit.color.g = 1.0
            hit.color.b = 0.62
            hit.color.a = 0.95
            markers.markers.append(hit)

        if bool(message.velocity_valid):
            velocity = message.velocity
            if all(
                math.isfinite(float(value))
                for value in (velocity.x, velocity.y, velocity.z)
            ):
                vector = (
                    self.velocity_scale_s * float(velocity.x),
                    self.velocity_scale_s * float(velocity.y),
                    self.velocity_scale_s * float(velocity.z),
                )
                speed_arrow = self._marker(message, "dvl_velocity", 0)
                speed_arrow.type = Marker.ARROW
                speed_arrow.scale.x = 0.030
                speed_arrow.scale.y = 0.075
                speed_arrow.scale.z = 0.095
                speed_arrow.color.r = 1.0
                speed_arrow.color.g = 0.38
                speed_arrow.color.b = 0.08
                speed_arrow.color.a = 1.0
                speed_arrow.points = [
                    _point(0.0, 0.0, 0.0),
                    _point(*vector),
                ]
                markers.markers.append(speed_arrow)

        accepted = dvl_sample_passes_input_gate(message)
        velocity = message.velocity
        lock = "INPUT OK" if accepted else "INPUT REJECTED"
        status_lines = (
            f"A50:{lock.replace(' ', '_')}|BEAMS:{valid_count}/4",
            f"ALT:{float(message.altitude):.2f}m|FOM:{float(message.fom):.4f}m/s",
            f"V:{float(velocity.x):+.3f}/{float(velocity.y):+.3f}/"
            f"{float(velocity.z):+.3f}m/s",
        )
        for index, text in enumerate(status_lines):
            status = self._marker(message, "dvl_status", index)
            status.type = Marker.TEXT_VIEW_FACING
            status.pose.position = _point(0.0, 0.0, -0.46 + 0.07 * index)
            status.scale.z = 0.032
            status.color.r = 0.22 if accepted else 1.0
            status.color.g = 1.0 if accepted else 0.28
            status.color.b = 0.58 if accepted else 0.12
            status.color.a = 1.0
            status.text = text
            markers.markers.append(status)
        self.marker_publisher.publish(markers)

    def _on_odometry(self, message: Odometry) -> None:
        stamp_s = (
            float(message.header.stamp.sec)
            + 1.0e-9 * float(message.header.stamp.nanosec)
        )
        frame_id = message.header.frame_id or "odom"
        if path_state_must_reset(
            self._last_path_stamp_s,
            self._path_frame_id,
            stamp_s,
            frame_id,
        ):
            self._path.clear()
            self._last_path_stamp_s = None
            self._last_path_position = None
            self.get_logger().warning(
                "Resetting filtered path after time reset or frame change."
            )
        self._path_frame_id = frame_id

        position = message.pose.pose.position
        current = (float(position.x), float(position.y), float(position.z))
        if self._last_path_stamp_s is not None and self._last_path_position is not None:
            elapsed = stamp_s - self._last_path_stamp_s
            distance = math.sqrt(
                sum(
                    (current[index] - self._last_path_position[index]) ** 2
                    for index in range(3)
                )
            )
            if elapsed < self.path_min_period_s and distance < self.path_min_distance_m:
                return

        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = frame_id
        pose.pose = message.pose.pose
        self._path.append(pose)
        self._last_path_stamp_s = stamp_s
        self._last_path_position = current

        path = Path()
        path.header = message.header
        path.header.frame_id = frame_id
        path.poses = list(self._path)
        self.path_publisher.publish(path)


def main(args: list[str] | None = None) -> None:
    """Run the focused DVL and localization visualizer."""
    rclpy.init(args=args)
    node = DvlLocalizationVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

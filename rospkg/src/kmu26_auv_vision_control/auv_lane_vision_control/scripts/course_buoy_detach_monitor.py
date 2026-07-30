#!/usr/bin/env python3
"""Publish unique, physically corroborated course-buoy detach events."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Iterator, Mapping, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String, UInt32


_ROW_CONTAINER_KEYS = ("buoys", "course_buoys", "targets", "items")
_NESTED_CONTAINER_KEYS = ("data", "payload", "status")
_ROW_HINT_KEYS = {
    "id",
    "target_id",
    "body_name",
    "has_magnet",
    "detached",
    "eq_active",
    "release_time_s",
}
_TRUE_TEXT = {"1", "true", "yes", "on"}
_FALSE_TEXT = {"0", "false", "no", "off"}


@dataclass(frozen=True)
class BuoyObservation:
    """Normalized status for one magnetic course buoy."""

    target_id: str
    detached: bool
    corroborated: bool


def _coerce_bool(value: Any) -> Optional[bool]:
    """Return a strict Boolean for common JSON encodings, otherwise None."""

    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        numeric = float(value)
        if math.isfinite(numeric) and numeric in (0.0, 1.0):
            return bool(int(numeric))
        return None
    if isinstance(value, str):
        normalized = value.strip().casefold()
        if normalized in _TRUE_TEXT:
            return True
        if normalized in _FALSE_TEXT:
            return False
    return None


def _coerce_finite_float(value: Any) -> Optional[float]:
    """Return a finite numeric JSON value, rejecting Booleans and junk."""

    if isinstance(value, bool) or value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return numeric if math.isfinite(numeric) else None


def _looks_like_row(value: Mapping[str, Any]) -> bool:
    return bool(_ROW_HINT_KEYS.intersection(value))


def _rows_from_container(container: Any) -> Iterator[Mapping[str, Any]]:
    """Yield rows from either a JSON list or an ID-keyed object."""

    if isinstance(container, list):
        for value in container:
            if isinstance(value, Mapping):
                yield value
        return

    if not isinstance(container, Mapping):
        return
    if _looks_like_row(container):
        yield container
        return

    for key, value in container.items():
        if not isinstance(value, Mapping):
            continue
        row = dict(value)
        row.setdefault("id", str(key))
        yield row


def iter_status_rows(
    payload: Any, depth: int = 0
) -> Iterator[Mapping[str, Any]]:
    """Accept the canonical payload plus common list/map wrapper variants."""

    if depth > 4:
        return
    if isinstance(payload, list):
        yield from _rows_from_container(payload)
        return
    if not isinstance(payload, Mapping):
        return

    found_container = False
    for key in _ROW_CONTAINER_KEYS:
        if key not in payload:
            continue
        found_container = True
        yield from _rows_from_container(payload[key])
    if found_container:
        return

    if _looks_like_row(payload):
        yield payload
        return

    for key in _NESTED_CONTAINER_KEYS:
        nested = payload.get(key)
        if isinstance(nested, (list, Mapping)):
            yield from iter_status_rows(nested, depth + 1)
            return

    # Also accept a top-level object keyed directly by physical target ID.
    yield from _rows_from_container(payload)


def _lookup(row: Mapping[str, Any], *keys: str) -> Any:
    """Look up a field directly or in common row-local status wrappers."""

    for key in keys:
        if key in row:
            return row[key]
    for wrapper_key in ("state", "status", "runtime"):
        wrapper = row.get(wrapper_key)
        if not isinstance(wrapper, Mapping):
            continue
        for key in keys:
            if key in wrapper:
                return wrapper[key]
    return None


def normalize_observation(row: Mapping[str, Any]) -> Optional[BuoyObservation]:
    """Normalize one row and exclude malformed or non-magnetic targets."""

    raw_id = _lookup(row, "id", "target_id", "body_name", "name")
    if raw_id is None or isinstance(raw_id, (dict, list, bool)):
        return None
    target_id = str(raw_id).strip()
    if not target_id:
        return None

    # detached=true is also the normal state for surface/non-magnetic floats.
    # Requiring an explicit true avoids counting them when a field is absent.
    if _coerce_bool(_lookup(row, "has_magnet", "magnetic")) is not True:
        return None

    detached = _coerce_bool(_lookup(row, "detached", "is_detached"))
    if detached is None:
        return None

    eq_active = _coerce_bool(
        _lookup(row, "eq_active", "equality_active", "magnet_eq_active")
    )
    release_time_s = _coerce_finite_float(
        _lookup(row, "release_time_s", "detach_time_s", "released_at_s")
    )
    corroborated = eq_active is False and (
        release_time_s is not None and release_time_s >= 0.0
    )
    return BuoyObservation(target_id, detached, corroborated)


class DetachEventTracker:
    """Find fresh false-to-true transitions and count each ID at most once."""

    def __init__(self) -> None:
        self._last_detached: dict[str, bool] = {}
        self._pending_rises: set[str] = set()
        self._counted_ids: set[str] = set()

    @property
    def count(self) -> int:
        return len(self._counted_ids)

    def update(self, payload: Any) -> list[str]:
        # Collapse duplicate aliases within one status frame before changing
        # edge state; a duplicate false/true pair must not manufacture an edge.
        observations: dict[str, BuoyObservation] = {}
        for row in iter_status_rows(payload):
            observation = normalize_observation(row)
            if observation is not None:
                observations[observation.target_id] = observation

        detached_ids: list[str] = []
        for target_id, observation in observations.items():
            previous = self._last_detached.get(target_id)
            self._last_detached[target_id] = observation.detached

            if not observation.detached:
                self._pending_rises.discard(target_id)
                continue
            if previous is False:
                self._pending_rises.add(target_id)

            if target_id in self._counted_ids:
                self._pending_rises.discard(target_id)
                continue
            if (
                target_id not in self._pending_rises
                or not observation.corroborated
            ):
                continue

            self._pending_rises.remove(target_id)
            self._counted_ids.add(target_id)
            detached_ids.append(target_id)

        return detached_ids


class CourseBuoyDetachMonitor(Node):
    """ROS wrapper around the simulator-neutral detach-event tracker."""

    def __init__(self) -> None:
        super().__init__("course_buoy_detach_monitor")
        self.declare_parameter("status_topic", "/mujoco/course_buoys/status")
        self.declare_parameter(
            "detach_count_topic", "/vision/course_buoy_detach_count"
        )
        self.declare_parameter(
            "detached_id_topic", "/vision/course_buoy_detached_id"
        )

        status_topic = self._required_topic("status_topic")
        detach_count_topic = self._required_topic("detach_count_topic")
        detached_id_topic = self._required_topic("detached_id_topic")

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=16,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._count_pub = self.create_publisher(
            UInt32, detach_count_topic, state_qos
        )
        self._id_pub = self.create_publisher(
            String, detached_id_topic, event_qos
        )
        self._status_sub = self.create_subscription(
            String, status_topic, self._on_status, 10
        )
        self._tracker = DetachEventTracker()

        self._publish_count()
        self.get_logger().info(
            f"Monitoring magnetic course buoys on {status_topic}; "
            f"publishing {detach_count_topic} and {detached_id_topic}"
        )

    def _required_topic(self, parameter_name: str) -> str:
        topic = str(self.get_parameter(parameter_name).value).strip()
        if not topic:
            raise ValueError(f"{parameter_name} must not be empty")
        return topic

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f"Ignoring invalid course-buoy status JSON: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        try:
            detached_ids = self._tracker.update(payload)
        except (TypeError, ValueError, OverflowError) as exc:
            self.get_logger().warning(
                f"Ignoring malformed course-buoy status payload: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        first_event_number = self._tracker.count - len(detached_ids) + 1
        for event_number, target_id in enumerate(
            detached_ids, start=first_event_number
        ):
            id_msg = String()
            id_msg.data = target_id
            self._id_pub.publish(id_msg)
            self.get_logger().info(
                f"Fresh magnetic detach #{event_number}: {target_id}"
            )
        if detached_ids:
            self._publish_count()

    def _publish_count(self) -> None:
        msg = UInt32()
        msg.data = self._tracker.count
        self._count_pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = CourseBuoyDetachMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

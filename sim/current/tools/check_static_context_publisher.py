#!/usr/bin/env python3
"""Smoke-check static context publish policy without ROS imports."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_static_context_publisher import StaticContextPublisher  # noqa: E402


class FakeString:
    def __init__(self) -> None:
        self.data = ""


def test_publish_cadence() -> None:
    calls: list[tuple[str, object]] = []

    def safe_publish(pub, msg, topic):
        calls.append((topic, msg))
        return True

    publisher = StaticContextPublisher(
        tf_static_pub=object(),
        robot_description_pub=object(),
        string_factory=FakeString,
        build_tf_message=lambda stamp, specs: {"stamp": stamp, "specs": specs},
        safe_publish=safe_publish,
        static_tf_specs=("base", "sensor"),
        robot_description_text="<robot/>",
        robot_description_pub_period_s=1.0,
    )

    assert publisher.publish("t0", 0.0)
    assert [topic for topic, _ in calls] == ["/tf_static", "/robot_description"]
    assert calls[1][1].data == "<robot/>"
    assert publisher.publish("t1", 0.5)
    assert [topic for topic, _ in calls] == ["/tf_static", "/robot_description"]
    assert publisher.publish("t2", 1.0)
    assert [topic for topic, _ in calls] == ["/tf_static", "/robot_description", "/robot_description"]


def test_static_tf_failure_keeps_unpublished() -> None:
    outcomes = [False, True]
    calls: list[str] = []

    def safe_publish(pub, msg, topic):
        calls.append(topic)
        return outcomes.pop(0)

    publisher = StaticContextPublisher(
        tf_static_pub=object(),
        robot_description_pub=None,
        string_factory=FakeString,
        build_tf_message=lambda stamp, specs: object(),
        safe_publish=safe_publish,
        static_tf_specs=("base",),
        robot_description_text="",
        robot_description_pub_period_s=1.0,
    )

    assert not publisher.publish("t0", 0.0)
    assert publisher.publish("t1", 0.1)
    assert calls == ["/tf_static", "/tf_static"]


def main() -> int:
    test_publish_cadence()
    test_static_tf_failure_keeps_unpublished()
    print("static_context_publisher=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

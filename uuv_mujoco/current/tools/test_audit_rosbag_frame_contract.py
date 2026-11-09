"""Check composed mount observations, ambiguity and lever-arm units."""

from __future__ import annotations

import unittest
import importlib.util
import sqlite3
import tempfile
from pathlib import Path

import numpy as np
from audit_rosbag_frame_contract import (
    resolve_transform,
    sensor_point_acceleration,
    sensor_point_velocity,
)


def edge(parent, xyz, quat=(0, 0, 0, 1)):
    return {"parent": parent, "xyz_m": list(xyz), "quat_xyzw": list(quat)}


class RecordedMountTests(unittest.TestCase):
    def test_compose_separate_tf_publications_with_rotation(self):
        edges = {
            "fcu": [edge("base_link", [1, 0, 0], [0, 0, 2**-0.5, 2**-0.5])],
            "imu": [edge("fcu", [1, 0, 0])],
        }
        result = resolve_transform(edges, "imu")
        self.assertEqual(result["status"], "recorded_static_tf")
        np.testing.assert_allclose(
            np.array(result["base_from_sensor"])[:3, 3], [1, 1, 0], atol=1e-12
        )

    def test_quaternion_sign_equivalent_publications_share_a_mount(self):
        edges = {"imu": [
            edge("base_link", [0.1, 0, 0]),
            edge("base_link", [0.1, 0, 0], [0, 0, 0, -1]),
        ]}
        self.assertEqual(resolve_transform(edges, "/imu")["status"], "recorded_static_tf")

    def test_conflicting_mount_needs_explicit_epoch(self):
        edges = {"imu": [
            edge("base_link", [0, 0, 0]), edge("base_link", [0.1, 0, 0]),
        ]}
        self.assertEqual(
            resolve_transform(edges, "imu")["status"], "conflicting_recorded_mount"
        )

    def test_dynamic_sensor_path_requires_time_resolution(self):
        edges = {
            "fcu": [edge("base_link", [0, 0, 0])],
            "imu": [edge("fcu", [0, 0, 0])],
        }
        result = resolve_transform(edges, "imu", dynamic_children={"fcu"})
        self.assertEqual(result["status"], "time_dependent_mount")

    def test_missing_and_cyclic_frames_do_not_become_zero_mounts(self):
        self.assertEqual(resolve_transform({}, "imu")["status"], "missing")
        edges = {
            "imu": [edge("fcu", [0, 0, 0])],
            "fcu": [edge("imu", [0, 0, 0])],
        }
        self.assertEqual(resolve_transform(edges, "imu")["status"], "cycle")

    def test_imu_lever_includes_tangential_and_centripetal_acceleration(self):
        actual = sensor_point_acceleration(
            np.zeros(3), [0, 0, 2], [0, 0, 3], [0.1, 0, 0]
        )
        np.testing.assert_allclose(actual, [-0.4, 0.3, 0])

    def test_velocity_at_offset_sensor_uses_rotation_cross_lever(self):
        actual = sensor_point_velocity(np.zeros(3), [0, 0, 2], [0.1, 0, 0])
        np.testing.assert_allclose(actual, [0, 0.2, 0])


@unittest.skipUnless(importlib.util.find_spec("rosbags"), "Optional rosbags decoder")
class RecordedBagTests(unittest.TestCase):
    def test_static_tf_fragments_are_composed_through_database_entry_point(self):
        from rosbags.typesys import Stores, get_typestore
        from audit_rosbag_frame_contract import audit

        store = get_typestore(Stores.ROS2_HUMBLE)
        types = store.types

        def fragment(parent, child, xyz):
            transform = types["geometry_msgs/msg/Transform"](
                types["geometry_msgs/msg/Vector3"](*xyz),
                types["geometry_msgs/msg/Quaternion"](0.0, 0.0, 0.0, 1.0),
            )
            header = types["std_msgs/msg/Header"](
                types["builtin_interfaces/msg/Time"](0, 0), parent
            )
            stamped = types["geometry_msgs/msg/TransformStamped"](
                header, child, transform
            )
            return bytes(store.serialize_cdr(
                types["tf2_msgs/msg/TFMessage"]([stamped]), "tf2_msgs/msg/TFMessage"
            ))

        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "fragments.db3"
            with sqlite3.connect(database) as connection:
                connection.executescript(
                    "CREATE TABLE topics(id INTEGER, name TEXT, type TEXT);"
                    "CREATE TABLE messages(id INTEGER, topic_id INTEGER, "
                    "timestamp INTEGER, data BLOB);"
                    "INSERT INTO topics VALUES(1,'/tf_static','tf2_msgs/msg/TFMessage');"
                )
                connection.executemany(
                    "INSERT INTO messages VALUES(?,1,?,?)",
                    [(1, 100, fragment("base_link", "fcu_link", [.11, 0., .09])),
                     (2, 200, fragment("fcu_link", "imu_link", [.01, 0., 0.]))],
                )
            before = database.read_bytes()
            result = audit(database)
            self.assertEqual(database.read_bytes(), before)
            imu = result["sensor_transforms"]["imu_link"]
            self.assertEqual(imu["status"], "recorded_static_tf")
            np.testing.assert_allclose(
                np.array(imu["base_from_sensor"])[:3, 3], [.12, 0., .09]
            )


if __name__ == "__main__":
    unittest.main()

"""Actual MuJoCo CAD fixtures; no renderer, ROS, policy, or learning required."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import json

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'uuv_mujoco/current'))
from sim.runtime.course_buoy_runtime import CourseBuoyRuntime
from bridge.ros2_publish_course_buoys import _runtime_buoy_row
from release_validation import validate_release
from reward import ForkReward


class ContactSnapshotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(
            ROOT / 'uuv_mujoco/current/scenes/research_pool_slam_scene.xml'))

    def setUp(self):
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.runtime = CourseBuoyRuntime.from_model(
            mujoco_module=mujoco, model=self.model, data=self.data, water_surface_z=0.,
            env_float=lambda name, default: default,
            env_flag=lambda name, default: False if name == 'UUV_COURSE_BUOY_TRACK_CSV_ENABLE' else default,
            log=lambda message: None)
        self.buoy = self.runtime.buoys[0]
        self.pipe = self.model.geom(self.buoy.name + '_pvc_pipe').id

    def place(self, point):
        b, d = self.buoy, self.data
        d.qpos[b.free_qposadr:b.free_qposadr+3] += point - d.geom_xpos[self.pipe]
        mujoco.mj_forward(self.model, d)
        self.runtime._contact_snapshot()

    def slot(self, offset=0.):
        root = self.runtime.vehicle_root_body_id
        return self.data.xpos[root] + self.data.xmat[root].reshape(3, 3) @ np.array(
            [.335511938+offset, -.091927344, -.104344226])

    def test_region_can_be_true_without_any_vehicle_contact(self):
        self.place(self.slot(.039))
        self.assertTrue(self.runtime.right_fork_region_contains(self.buoy))
        evidence = self.runtime.release_contact_snapshot(self.buoy)
        self.assertEqual(evidence['contacts'], [])
        # An externally triggered release at this geometry fooled the old
        # reward. Preserve that old contract and explicitly reject it in v1.
        initial = dict(time_s=0., target_id='rod', detached=False, eq_active=True,
                       height_error_m=0., fork_stem_distance_m=.039,
                       camera_valid=False, visible=False, depth_m=1., yaw_rad=0.)
        reward = ForkReward()
        reward.reset(initial)
        self.data.time = 1.
        self.runtime._detach(self.buoy, reason='diagnostic_external', force_n=21.)
        measure = dict(initial, time_s=1., detached=True, eq_active=False, release_time_s=1.,
                       release_right_fork_region=self.buoy.release_right_fork_region,
                       release_right_fork_stem_contact=self.buoy.release_right_fork_stem_contact,
                       release_contact_evidence=self.buoy.release_contact_evidence)
        self.assertTrue(reward.step(measure)['success'])
        result = validate_release(measure, released=True)
        self.assertFalse(result['contact_and_stability_verified'])
        self.assertEqual(result['contact_status'], 'rejected')

    def test_right_contact_force_position_and_frozen_release(self):
        self.place(self.slot())
        before = self.runtime.release_contact_snapshot(self.buoy)
        self.assertTrue(any(c['kind'] == 'right_fork_stem' and c['force_n'] > 0
                            for c in before['contacts']))
        for c in before['contacts']:
            self.assertEqual(len(c['position_world_m']), 3)
            self.assertTrue(np.isfinite(c['force_n']))
        self.data.time = 1.
        self.runtime._detach(self.buoy, reason='diagnostic_contact', force_n=21.)
        evidence = self.buoy.release_contact_evidence
        self.assertEqual(evidence['time_s'], self.buoy.release_time_s)
        row = _runtime_buoy_row(SimpleNamespace(_course_buoy_runtime=self.runtime),
                               self.buoy.name, self.buoy.name + '_float')
        self.assertEqual(json.loads(json.dumps(row))['release_contact_evidence'], evidence)
        self.place(self.slot(1.))
        self.assertIs(self.buoy.release_contact_evidence, evidence)
        self.assertTrue(evidence['contacts'])

    def test_other_hand_and_hull_contacts_not_right_stem(self):
        initial = self.data.qpos.copy()
        for label, geoms in (
            ('other_hand', self.runtime._hand_geom_id_set - self.runtime._right_fork_geom_ids),
            ('vehicle_body', self.runtime.vehicle_geom_ids - self.runtime._hand_geom_id_set),
        ):
            found = False
            for gid in sorted(geoms):
                self.data.qpos[:] = initial
                mujoco.mj_forward(self.model, self.data)
                self.place(self.data.geom_xpos[gid].copy())
                contacts = self.runtime.release_contact_snapshot(self.buoy)['contacts']
                if any(c['kind'] == label and c['force_n'] > 0 for c in contacts):
                    found = True
                    break
            self.assertTrue(found, label)

    def test_legacy_contact_scanner_agrees_with_single_pass(self):
        self.place(self.slot())
        bodies, _, _ = self.runtime._contact_snapshot()
        self.assertEqual(self.runtime._vehicle_contacted_buoy_body_ids(), bodies)


if __name__ == '__main__':
    unittest.main()

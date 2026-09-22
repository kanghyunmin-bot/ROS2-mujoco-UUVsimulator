"""Check that physical release evidence remains latched after contact ends."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'uuv_mujoco/current'))
from sim.runtime.course_buoy_runtime import CourseBuoyRuntime


class ReleaseEvidence(unittest.TestCase):
    def test_hull_release_policy_does_not_imply_hand_contact(self):
        import numpy as np
        runtime = object.__new__(CourseBuoyRuntime)
        runtime._buoy_body_by_geom = {9: 90}
        runtime.vehicle_geom_ids = frozenset([1, 2])
        runtime._release_probe_geom_id_set = frozenset([1, 2])
        runtime._hand_geom_id_set = frozenset([2])
        runtime._hand_contacted_body_ids = set()
        runtime._contact_force_scratch = np.zeros(6)
        runtime.mujoco_module = None
        runtime.data = SimpleNamespace(ncon=1, contact=[SimpleNamespace(geom1=1, geom2=9)])
        self.assertIn(90, runtime._contact_snapshot()[1])
        self.assertEqual(runtime._hand_contacted_body_ids, set())
        runtime.data.contact[0].geom1 = 2
        runtime._contact_snapshot()
        self.assertEqual(runtime._hand_contacted_body_ids, {90})
        runtime.data.ncon = 0
        runtime._contact_snapshot()
        self.assertEqual(runtime._hand_contacted_body_ids, set())

    def test_release_evidence_is_immutable(self):
        runtime = object.__new__(CourseBuoyRuntime)
        runtime.data = SimpleNamespace(time=12., eq_active=[1, 1])
        runtime.magnet_force_release = False
        runtime.log = lambda _: None
        for name in ('_restore_buoy_collisions', '_suppress_flex_line_collisions',
                     '_recompute_after_release', '_hide_surface_projection'):
            setattr(runtime, name, lambda _: None)
        buoy = SimpleNamespace(detached=False, rake_contact_active=True, hand_contact_active=True,
                               eq_id=0, flex_line_top_eq_id=1, name='test')
        runtime._detach(buoy, reason='rake_contact_immediate', force_n=4.)
        self.assertEqual(runtime.data.eq_active, [0, 0])
        self.assertTrue(buoy.release_rake_contact)
        self.assertTrue(buoy.release_hand_contact)
        self.assertEqual(buoy.release_time_s, 12.)
        self.assertEqual(buoy.release_force_n, 4.)
        buoy.rake_contact_active = False
        buoy.hand_contact_active = False
        runtime.data.time = 13.
        runtime._detach(buoy, reason='other', force_n=8.)
        self.assertTrue(buoy.release_rake_contact)
        self.assertTrue(buoy.release_hand_contact)
        self.assertEqual(buoy.release_reason, 'rake_contact_immediate')
        self.assertEqual(buoy.release_time_s, 12.)


if __name__ == '__main__':
    unittest.main()

import unittest
from outcome import classify_release


class ReleaseLabels(unittest.TestCase):
    def setUp(self):
        self.row = dict(id='target', detached=True, eq_active=False,
                        release_time_s=12., release_reason='rake_contact_immediate',
                        release_rake_contact=True, release_hand_contact=True)

    def test_contact_and_weld_release(self):
        self.assertTrue(classify_release({'target'}, [self.row], 10.)['success'])

    def test_hull_contact_is_not_hand_contact(self):
        self.row['release_hand_contact'] = False
        self.assertFalse(classify_release({'target'}, [self.row], 10.)['success'])

    def test_missing_hand_evidence_is_not_success(self):
        del self.row['release_hand_contact']
        self.assertFalse(classify_release({'target'}, [self.row], 10.)['success'])

    def test_proximity_is_not_contact(self):
        self.row['release_rake_contact'] = False
        self.assertFalse(classify_release({'target'}, [self.row], 10.)['success'])

    def test_disappearance_is_not_success(self):
        self.assertIsNone(classify_release({'target'}, [], 10.))

    def test_old_release_and_other_target_are_ignored(self):
        self.assertIsNone(classify_release({'target'}, [self.row], 13.))
        self.assertIsNone(classify_release({'other'}, [self.row], 10.))

    def test_active_weld_is_not_success(self):
        self.row['eq_active'] = True
        self.assertFalse(classify_release({'target'}, [self.row], 10.)['success'])

    def test_missing_contact_evidence_is_not_success(self):
        del self.row['release_rake_contact']
        self.assertFalse(classify_release({'target'}, [self.row], 10.)['success'])


if __name__ == '__main__':
    unittest.main()

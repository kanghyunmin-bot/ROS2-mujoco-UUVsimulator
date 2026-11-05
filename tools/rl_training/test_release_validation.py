"""Release verification must reject geometry-only and incomplete evidence."""
import copy
import unittest

from release_validation import validate_release


def event():
    return dict(release_time_s=1., release_right_fork_region=True,
                release_right_fork_stem_contact=True,
                release_contact_evidence=dict(schema="uuv.release_contact.v1", time_s=1., contacts=[
                    dict(kind="right_fork_stem", solver_active=True, force_n=2.)]))


def stable_samples():
    return [dict(time_s=1.+i*.1, tilt_rad=.1, angular_speed_rad_s=.2) for i in range(21)]


class ReleaseValidationTests(unittest.TestCase):
    def test_complete_contact_and_stability(self):
        result = validate_release(event(), released=True, post_release=stable_samples())
        self.assertTrue(result['contact_and_stability_verified'])

    def test_contact_alone_does_not_prove_stability(self):
        result = validate_release(event(), released=True)
        self.assertEqual(result['contact_status'], 'verified')
        self.assertFalse(result['contact_and_stability_verified'])

    def test_missing_old_evidence_is_unknown(self):
        measure = event()
        del measure['release_contact_evidence']
        self.assertEqual(validate_release(measure, released=True)['contact_status'], 'unknown')

    def test_region_without_contact(self):
        measure = event()
        measure['release_contact_evidence']['contacts'] = []
        result = validate_release(measure, released=True, post_release=stable_samples())
        self.assertFalse(result['contact_and_stability_verified'])
        self.assertIn('no_loaded_right_fork_stem_contact', result['reasons'])

    def test_other_contact_and_inactive_force(self):
        for kind in ('other_hand', 'vehicle_body', 'right_fork_other'):
            measure = event()
            measure['release_contact_evidence']['contacts'].append(
                dict(kind=kind, solver_active=True, force_n=1.))
            self.assertIn('simultaneous_other_contact', validate_release(measure, released=True)['reasons'])
        for change in ({'force_n': 0.}, {'solver_active': False}):
            measure = event()
            measure['release_contact_evidence']['contacts'][0].update(change)
            self.assertEqual(validate_release(measure, released=True)['contact_status'], 'rejected')

    def test_invalid_evidence(self):
        for change in ({'time_s': .9}, {'time_s': float('nan')}, {'contacts': None},
                       {'contacts': [{'force_n': float('nan')}]}, {'contacts': [None]}):
            measure = event()
            measure['release_contact_evidence'].update(change)
            self.assertEqual(validate_release(measure, released=True)['contact_status'], 'unknown')

    def test_stability_gaps_truncation_and_unstable_motion(self):
        for samples in ([], stable_samples()[:5], stable_samples()[4:],
                        stable_samples()[:3] + stable_samples()[8:]):
            result = validate_release(event(), released=True, post_release=samples)
            self.assertFalse(result['contact_and_stability_verified'])
            self.assertEqual(result['stability_status'], 'unknown')
        for key in ('tilt_rad', 'angular_speed_rad_s'):
            samples = stable_samples()
            samples[10][key] = 1.
            self.assertEqual(validate_release(event(), released=True, post_release=samples)['stability_status'], 'unstable')

    def test_inputs_unchanged_and_release_required(self):
        measure, samples = event(), stable_samples()
        saved = copy.deepcopy((measure, samples))
        self.assertFalse(validate_release(measure, released=False, post_release=samples)['contact_and_stability_verified'])
        self.assertEqual((measure, samples), saved)


if __name__ == '__main__':
    unittest.main()

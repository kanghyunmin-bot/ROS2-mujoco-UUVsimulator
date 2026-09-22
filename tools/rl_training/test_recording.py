import unittest
from recording import finish_recording

class RecordingTest(unittest.TestCase):
    def run_case(self, active, saved, race=False):
        state = {'active': active, 'busy': False, 'last_result': saved}
        calls = []
        def api(command=None, **kw):
            if command:
                calls.append(kw['action'])
                state['active'] = False
                if race:
                    raise RuntimeError('녹화 중인 시연이 없습니다.')
            return {'recorder': state}
        def wait(fn, label):
            assert fn()
        result = finish_recording(api, wait, previous_path='old', success=True)
        return result, calls

    def test_automatic_save_does_not_stop_twice(self):
        result, calls = self.run_case(False, dict(path='new', frames=286, success=False, termination_reason='sampling_discontinuity'))
        self.assertFalse(result[1])
        self.assertEqual(calls, [])

    def test_stop_race_accepts_verified_automatic_save(self):
        result, calls = self.run_case(True, dict(path='new', frames=286, success=False, termination_reason='sampling_discontinuity'), True)
        self.assertFalse(result[1])

    def test_normal_success(self):
        result, calls = self.run_case(True, dict(path='new', frames=10, success=True, termination_reason='operator_stop'))
        self.assertTrue(result[1])
        self.assertEqual(calls, ['success'])

    def test_old_result_rejected(self):
        with self.assertRaises(RuntimeError):
            self.run_case(False, dict(path='old', frames=10, success=False, termination_reason='sampling_discontinuity'))

    def test_other_failure_rejected(self):
        with self.assertRaises(RuntimeError):
            self.run_case(False, dict(path='new', frames=10, success=False, termination_reason='sensor_stale'))

if __name__ == '__main__':
    unittest.main()

import os,tempfile,unittest
from pathlib import Path
from recovery import physics_failure, FailureIsolation

class Tests(unittest.TestCase):
    def test_original_crash_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'mujoco_1.log'
            p.write_text('WARNING: Nan, Inf or huge value in QACC at DOF 28.\nRuntimeError: Physics instability at 453.301500 s; stopped instead of continuing after an automatic reset. Previous state: saved.npz\n')
            self.assertEqual(physics_failure(tmp)['kind'],'physics_instability')
            self.assertIn('453.301500',physics_failure(tmp)['detail'])
            newer=Path(tmp)/'mujoco_2.log';newer.write_text('healthy runtime')
            os.utime(newer,ns=(p.stat().st_mtime_ns+1000,p.stat().st_mtime_ns+1000))
            self.assertIsNone(physics_failure(tmp))
    def test_discard_independent_worker_and_bound_repetition(self):
        b=FailureIsolation()
        b.discard(1,'physics_instability')
        self.assertEqual(b.retired,{1})
        b.discard(3,'simulation_tick_timeout')
        self.assertEqual(b.retired,{1,3})
        with self.assertRaisesRegex(RuntimeError,'3개 환경'):
            b.discard(4,'worker_exit')
    def test_code_errors_are_not_silently_dropped(self):
        b=FailureIsolation()
        with self.assertRaisesRegex(RuntimeError,'학습 코드 오류'):
            b.discard(0,'unknown')
        self.assertFalse(b.retired)

if __name__=='__main__':unittest.main()

"""Input integrity checks for the offline yaw experiment."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from yaw_release_trace import validate_trace

class TraceTests(unittest.TestCase):
    def fixture(self):
        return dict(release_s=.5,initial_yaw_rate_flu_radps=.2, rows=[
            dict(t_s=t,servo_pwm=[1500]*8,rc4_pwm=1500,observed_yaw_rate_flu_radps=.2)
            for t in (0.,1.)])
    def test_preserves_final_pwm(self):
        trace=self.fixture();trace['rows'][0]['servo_pwm']=[1510,1490,1520,1480,1500,1500,1500,1500]
        original=deepcopy(trace)
        self.assertEqual(validate_trace(trace),original)
    def test_rejects_invalid_clock_and_pwm(self):
        for bad in ('duplicate','backward','missing','invalid','nonfinite'):
            trace=self.fixture()
            if bad=='duplicate':trace['rows'][1]['t_s']=0.
            if bad=='backward':trace['rows'][1]['t_s']=-1.
            if bad=='missing':trace['rows'][0]['servo_pwm'].pop()
            if bad=='invalid':trace['rows'][0]['servo_pwm'][0]=65535
            if bad=='nonfinite':trace['rows'][0]['servo_pwm'][0]=float('nan')
            with self.assertRaises(ValueError):validate_trace(trace)
    def test_release_must_be_interior(self):
        for release in (-1.,0.,1.,2.):
            trace=self.fixture();trace['release_s']=release
            with self.assertRaises(ValueError):validate_trace(trace)

if __name__=='__main__':unittest.main()

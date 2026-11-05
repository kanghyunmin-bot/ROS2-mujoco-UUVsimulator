"""Precision v4 configuration is explicit; older runs keep their semantics."""
import copy
import tempfile
import unittest
from pathlib import Path
from rl_config import defaults, validate


class ConfigTests(unittest.TestCase):
    def test_v4_bounds_seed_and_explicit_groups(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint=Path(directory)/'seed.pt';checkpoint.touch()
            data=defaults()
            data.update(models=['base'], resume_candidate=True, reward_version='precision_v4',
                        spawn_region=dict(seed=42,x_m=[-1.5,1.5],y_m=[-1,1]),
                        policy_groups={'policy':dict(base_model='base',environments=5,checkpoint=str(checkpoint))})
            self.assertEqual(validate(data,{'base':{}}),data)
            for region in (None,dict(seed=-1,x_m=[-1,1],y_m=[-1,1]),
                           dict(seed=True,x_m=[-1,1],y_m=[-1,1]),
                           dict(seed=42,x_m=[-3,1],y_m=[-1,1])):
                bad=copy.deepcopy(data);bad['spawn_region']=region
                with self.assertRaises(ValueError):validate(bad,{'base':{}})
            data['reward_version']='precision_v3'
            with self.assertRaises(ValueError):validate(data,{'base':{}})
            data.pop('spawn_region')
            self.assertIsNone(validate(data,{'base':{}})['spawn_region'])


if __name__=='__main__':unittest.main()

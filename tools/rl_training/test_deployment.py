"""Inference parity and incompatible-checkpoint checks without robot commands."""
import tempfile
import unittest
from pathlib import Path

import numpy as np
import torch

from deployment import ResidualDeployment, STATE_KEYS
from residual_policy import ResidualActorCritic


class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.base = self.root / 'base'
        self.actor = ResidualActorCritic()
        with torch.no_grad():
            self.actor.actor.bias.copy_(torch.tensor([.5, -.5, 1., -1.]))
        self.checkpoint = self.root / 'candidate.pt'
        torch.save(dict(schema='uuv.residual_rl.v1', base_model='base', updates=15,
                        residual_limit=.15, policy=self.actor.state_dict()), self.checkpoint)
        self.deployment = ResidualDeployment(self.checkpoint, self.base, self.root)
        widths = (4, 3, 3, 3, 3, 1, 1, 5)
        self.obs = {'state.' + k: np.zeros((1, n)) for k, n in zip(STATE_KEYS, widths)}

    def test_matches_rollout_first_action_for_every_legacy_client_index(self):
        base = np.linspace(-1, 1, 64, dtype=np.float32).reshape(16, 4)
        original = base.copy()
        expected = self.actor.act(torch.zeros(1, 23), torch.from_numpy(base[:1]),
                                  deterministic=True)[0].numpy()[0]
        for motion in (base, base[None]):
            actual = self.deployment.apply(self.obs, motion)
            self.assertEqual(actual.shape, motion.shape)
            np.testing.assert_allclose(actual.reshape(16, 4), np.tile(expected, (16, 1)))
            np.testing.assert_array_equal(actual, self.deployment.apply(self.obs, motion))
        self.assertGreater(np.max(np.abs(expected-base[0])), .01)
        np.testing.assert_array_equal(base, original)

    def test_rejects_wrong_base_checkpoint(self):
        with self.assertRaisesRegex(ValueError, 'different base'):
            ResidualDeployment(self.checkpoint, self.root/'wrong', self.root)

    def test_v2_brake_deployment_parity(self):
        actor=ResidualActorCritic(learned_brake=True)
        with torch.no_grad():actor.actor.bias[4]=2.
        torch.save(dict(schema='uuv.residual_rl.v2',learned_brake=True,
                        base_model='base',updates=1,residual_limit=.15,
                        policy=actor.state_dict()),self.checkpoint)
        deployed=ResidualDeployment(self.checkpoint,self.base,self.root)
        base=np.ones((16,4),dtype=np.float32)
        actual=deployed.apply(self.obs,base)
        expected=actor.act(torch.zeros(1,23),torch.ones(1,4),True)[0].numpy()
        np.testing.assert_allclose(actual,np.repeat(expected,16,axis=0))
        self.assertLess(actual[0,0],.04)

    def test_rejects_bad_observation_and_nonfinite_motion(self):
        base = np.zeros((16, 4))
        self.obs['state.depth'] = np.array([[np.nan]])
        with self.assertRaisesRegex(ValueError, '23-dimensional'):
            self.deployment.apply(self.obs, base)
        base[0, 0] = np.nan
        with self.assertRaisesRegex(ValueError, 'finite'):
            self.deployment.apply(self.obs, base)


if __name__ == '__main__':
    unittest.main()

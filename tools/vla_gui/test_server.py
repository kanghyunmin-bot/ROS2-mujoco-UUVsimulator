"""Control gates tested without starting or commanding a robot."""
import unittest
from unittest.mock import Mock, patch
import server


class ManagerTests(unittest.TestCase):
    def test_ppo_candidate_keeps_base_and_residual_paths_separate(self):
        server.refresh_new_model()
        candidate = server.MODELS[server.PPO_MODEL_KEY]
        self.assertEqual(candidate['path'], server.MODELS[server.NEW_MODEL_KEY]['path'])
        self.assertTrue(candidate['residual_checkpoint'].endswith('collection-00046.pt'))
        self.assertEqual(server.DEFAULT_MODEL, server.PPO_MODEL_KEY)
        with self.assertRaisesRegex(ValueError, 'PPO 실행 후보'):
            server.validate({**server.defaults(), 'models': [server.PPO_MODEL_KEY]}, server.MODELS)

    def test_rejects_unknown_checkpoint_before_side_effects(self):
        manager = server.Manager()
        with patch.object(server, 'container') as find:
            with self.assertRaises(ValueError):
                manager.prepare('../../other-model')
            find.assert_not_called()

    def test_model_exit_revokes_readiness(self):
        manager = server.Manager()
        manager.update(phase='ready', model='3-40000')
        manager.children['model'] = Mock(poll=Mock(return_value=1))
        self.assertEqual(manager.snapshot()['phase'], 'error')
        with self.assertRaises(RuntimeError):
            manager.run(50, 'task')

    def test_commands_are_serialized(self):
        manager = server.Manager()
        manager.update(busy=True)
        with self.assertRaises(ValueError):
            manager.submit('prepare', {'model': '1-20000'})
        manager.submit('stop', {})
        self.assertTrue(manager.cancel.is_set())

    def test_rejects_real_or_missing_sim_before_arm(self):
        manager = server.Manager()
        manager.name = 'sim-container'
        manager.update(phase='ready', model='1-20000')
        manager.children['model'] = Mock(poll=Mock(return_value=None))
        with patch.object(server, 'container', return_value='sim-container'), \
             patch.object(server.subprocess, 'run', return_value=Mock(returncode=1)), \
             patch.object(server, 'command') as command:
            with self.assertRaisesRegex(RuntimeError, 'MuJoCo'):
                manager.run(50, 'task')
            command.assert_not_called()

    def test_rejects_invalid_duration_before_arm(self):
        manager = server.Manager()
        manager.update(phase='ready', model='1-20000')
        manager.children['model'] = Mock(poll=Mock(return_value=None))
        for seconds in (0, 91, float('nan')):
            with self.assertRaises(ValueError):
                manager.run(seconds, 'task')


if __name__ == '__main__':
    unittest.main()

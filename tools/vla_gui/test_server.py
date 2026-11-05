"""Control gates tested without starting or commanding a robot."""
import unittest
import io
from unittest.mock import Mock, patch
import server


class ManagerTests(unittest.TestCase):
    def test_manager_reaches_the_configured_live_gui_port(self):
        with patch.dict(server.os.environ, {'UUV_GUI_URL': 'http://127.0.0.1:18879/'}), \
             patch.object(server, 'urlopen', return_value=io.BytesIO(b'{"ok":true}')) as open_url:
            self.assertEqual(server.request('/api/status'), {'ok': True})
        self.assertEqual(open_url.call_args.args[0].full_url, 'http://127.0.0.1:18879/api/status')

    def test_general_candidate_uses_evaluated_collection_policy(self):
        server.refresh_new_model()
        candidate = server.MODELS['success-135-general-35']
        self.assertEqual(candidate['path'], server.MODELS[server.NEW_MODEL_KEY]['path'])
        self.assertEqual(candidate['residual_checkpoint'],
                         'outputs/rl-training/20260922-155955-6c1b18/precision-candidate/collection-00035.pt')
        self.assertIn('4/4', candidate['description'])
        self.assertIn('미승인', candidate['description'])
        with self.assertRaisesRegex(ValueError, 'PPO 실행 후보'):
            server.validate({**server.defaults(), 'models': ['success-135-general-35']}, server.MODELS)

    def test_precision_candidate_uses_recorded_collection_policy(self):
        server.refresh_new_model()
        candidate = server.MODELS['success-135-precision-59']
        self.assertEqual(candidate['path'], server.MODELS[server.NEW_MODEL_KEY]['path'])
        self.assertEqual(candidate['residual_checkpoint'],
                         'outputs/rl-training/20260922-155955-6c1b18/release-best/collection-00059.pt')
        self.assertIn('0.856', candidate['description'])
        self.assertIn('성공률이 아닙니다', candidate['description'])
        self.assertTrue(candidate['ready'])
        with self.assertRaisesRegex(ValueError, 'PPO 실행 후보'):
            server.validate({**server.defaults(), 'models': ['success-135-precision-59']}, server.MODELS)

    def test_precision_candidate_is_hidden_when_checkpoint_is_missing(self):
        missing = Mock(is_file=Mock(return_value=False))
        try:
            with patch.object(server, 'PRECISION_CHECKPOINT', missing):
                server.refresh_new_model()
                self.assertNotIn('success-135-precision-59', server.MODELS)
        finally:
            server.refresh_new_model()

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

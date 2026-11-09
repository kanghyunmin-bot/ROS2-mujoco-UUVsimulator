"""Do not mix zero-based rollout directories with checkpoint collections."""
import json
from pathlib import Path
import tempfile
import unittest

from audit_precision import audit_run


class AuditTests(unittest.TestCase):
    def test_saved_rounds_follow_runner_numbering(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'config.json').write_text(json.dumps({'evaluation_repeats': 2, 'reward_version': 'precision_v4'}))
            for wave in (0, 1, 2, 3, 4, 5):
                folder = root/'env-0'/'outputs'/f'rollout-{wave:05d}'
                folder.mkdir(parents=True)
                (folder/'episode.json').write_text(json.dumps({'trace': [], 'released': False, 'release_tier': 'none'}))
            report = audit_run(root)
            self.assertEqual([e['collection'] for e in report['episodes']], [1, 2, 3, 4, 5, 6])
            self.assertEqual([e['role'] for e in report['episodes']], [
                'baseline_evaluation', 'training', 'incumbent_evaluation',
                'incumbent_evaluation', 'candidate_evaluation', 'candidate_evaluation'])


if __name__ == '__main__':
    unittest.main()

"""Model discovery works from a fresh checkout without local experiment folders."""
import json
import tempfile
import unittest
from pathlib import Path
from runtime_config import load_models

class DiscoveryTests(unittest.TestCase):
    def test_collection_install_needs_no_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(load_models(Path(tmp)), ({}, None))

    def test_missing_shard_is_not_offered_then_complete_model_is(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            model = root / 'models/fork-medium-10000'
            model.mkdir(parents=True)
            (model / 'config.json').write_text('{}')
            (model / 'model.safetensors.index.json').write_text(json.dumps({'weight_map': {'w':'model.safetensors'}}))
            self.assertEqual(load_models(root), ({}, None))
            (model / 'model.safetensors').write_bytes(b'test')
            (model / 'model_info.json').write_text(json.dumps({'label':'Latest','task':'custom task'}))
            models, default = load_models(root)
            self.assertEqual(default, 'fork-medium-10000')
            self.assertEqual(models[default]['task'], 'custom task')
            self.assertEqual(models[default]['path'], 'models/fork-medium-10000')

    def test_path_traversal_shard_is_not_a_complete_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=root/'models/test';m.mkdir(parents=True)
            (root/'secret').write_text('not a shard')
            (m/'config.json').write_text('{}')
            (m/'model.safetensors.index.json').write_text(json.dumps({'weight_map':{'w':'../../secret'}}))
            self.assertEqual(load_models(root),({},None))

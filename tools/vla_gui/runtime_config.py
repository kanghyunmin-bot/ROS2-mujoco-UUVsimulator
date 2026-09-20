"""Portable model discovery; the collector does not require a VLA installation."""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TASK = 'Approach the yellow buoy, align the fixed fork, and detach the buoy.'
U0_ROOT = Path(os.environ.get('UUV_U0_ROOT', str(ROOT / 'external/auv_vla'))).expanduser()
HOST_PYTHON = os.environ.get('UUV_VLA_PYTHON')
CONTAINER_PYTHON = '/workspace/.venv-vla/bin/python'


def load_models(root=ROOT):
    models = {}
    for folder in sorted((root / 'models').glob('*')):
        if folder.name.startswith('.'):
            continue
        index = folder / 'model.safetensors.index.json'
        if not index.is_file() or not (folder / 'config.json').is_file():
            continue
        try:
            shards = set(json.loads(index.read_text())['weight_map'].values())
            if not shards or not all((folder / s).is_file() and (folder / s).resolve().is_relative_to(folder.resolve()) for s in shards):
                continue
            info_path = folder / 'model_info.json'
            info = json.loads(info_path.read_text()) if info_path.exists() else {}
            models[folder.name] = dict(label=info.get('label', folder.name), path=str(folder.relative_to(root)),
                                       task=info.get('task', DEFAULT_TASK))
        except (OSError, ValueError, KeyError, TypeError):
            continue
    default = 'fork-medium-10000' if 'fork-medium-10000' in models else next(iter(models), None)
    return models, default

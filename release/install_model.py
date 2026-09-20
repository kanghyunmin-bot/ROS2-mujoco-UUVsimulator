"""Copy inference-only checkpoint files, preserving the source model."""
import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('checkpoint', type=Path)
a = p.parse_args()
source = a.checkpoint.expanduser().resolve()
index = json.loads((source / 'model.safetensors.index.json').read_text())
config = json.loads((source / 'config.json').read_text())
if config.get('action_head_cfg', {}).get('target_loss_weight') != 0:
    raise SystemExit('Expected a CAP-free KMU26 checkpoint.')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rospkg/src/auv_vla_data_collector'))
from kmu26_auv_vla_data_collector.deployment_config import validate_deployment_contract
validate_deployment_contract(source)
files = ['config.json', 'model.safetensors.index.json', 'experiment_cfg/kmu26_transfer.json', 'experiment_cfg/metadata.json']
files += sorted(set(index['weight_map'].values()))
files += [p.name for p in source.glob('LICENSE*.txt')]
if (source/'MODEL_CARD.md').is_file():files.append('MODEL_CARD.md')
for name in files:
    path = source / name
    if not path.resolve().is_relative_to(source) or not path.is_file():
        raise SystemExit(f'Missing/invalid checkpoint file: {name}')
root = Path(__file__).resolve().parents[1]
dest = root / 'models' / source.name
if dest.exists():
    raise SystemExit(f'Already exists (not overwritten): {dest}')
dest.parent.mkdir(parents=True, exist_ok=True)
staging = Path(tempfile.mkdtemp(prefix='.install-', dir=dest.parent))
for name in files:
    target = staging / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source / name, target)
if (source / 'model_info.json').exists():
    shutil.copy2(source / 'model_info.json', staging / 'model_info.json')
staging.rename(dest)
print(dest)

"""Warm the HTTP model with recorded observations without publishing robot commands."""
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'outputs/vla-transfer-audit-20260910/upstream/auv_vla'),
                str(ROOT / 'rospkg/src/auv_vla_data_collector'),
                str(ROOT / 'rospkg/src/kmu26_auv_vla_policy')]
import json_numpy
import requests
from kmu26_auv_vla_data_collector.transfer_config import Kmu26TransferDataConfig
from kmu26_auv_vla_data_collector.training_input import Kmu26TrainingDataset
from kmu26_auv_vla_policy.kmu26_contract import motion_chunk

config = Kmu26TransferDataConfig()
dataset = Kmu26TrainingDataset(
    dataset_path=str(ROOT / 'outputs/vla-first-training-20260920/validation'),
    modality_configs=config.modality_config(), embodiment_tag='new_embodiment', video_backend='decord')
sample = {k: v for k, v in dataset[0].items() if not k.startswith('action.')}
payload = {'encoded': json_numpy.dumps({'observation': sample})}
times = []
for _ in range(5):
    start = time.monotonic()
    response = requests.post('http://127.0.0.1:8000/act', json=payload, timeout=30)
    response.raise_for_status()
    motion_chunk(json_numpy.loads(response.text))
    times.append(time.monotonic() - start)
if max(times[-3:]) >= 0.25:
    raise RuntimeError(f'예열 후 응답이 제어 기한에 비해 느립니다: {times[-3:]}')
print(json.dumps({'warmup_ms': round(max(times[-3:]) * 1000)}), flush=True)

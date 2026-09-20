"""Warm HTTP inference with synthetic inputs; no dataset or robot action required."""
import json
import sys
import time
from pathlib import Path
import numpy as np
import json_numpy
import requests

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'rospkg/src/kmu26_auv_vla_policy'))
from kmu26_auv_vla_policy.kmu26_contract import motion_chunk

sample = {f'video.{name}': np.zeros((1, 360, 640, 3), dtype=np.uint8) for name in ('ego', 'buoy_release')}
for name, size in [('prev_command',4),('dvl_velocity',3),('angular_velocity',3),
                   ('linear_acceleration',3),('attitude',4),('depth',1),('altitude',1),('validity',4)]:
    sample[f'state.{name}'] = np.zeros((1,size),dtype=np.float32)
sample['state.attitude'][0,0] = 1.
sample['state.validity'][:] = 1.
sample['annotation.human.action.task_description'] = ['Approach the yellow buoy, align the fixed fork, and detach the buoy.']
payload = {'encoded': json_numpy.dumps({'observation': sample})}
times = []
for _ in range(5):
    start = time.monotonic()
    response = requests.post('http://127.0.0.1:8000/act', json=payload, timeout=60)
    response.raise_for_status()
    motion_chunk(json_numpy.loads(response.text))
    times.append(time.monotonic() - start)
if max(times[-3:]) >= .25:
    raise RuntimeError(f'Inference too slow for the 0.3s control deadline: {times[-3:]}')
print(json.dumps({'warmup_ms': round(max(times[-3:]) * 1000)}), flush=True)

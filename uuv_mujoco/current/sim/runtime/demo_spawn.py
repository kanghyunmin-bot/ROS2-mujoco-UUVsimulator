"""Explicit, reproducible central-pool target placement for isolated RL resets."""
import json
import math
import os
from pathlib import Path
import random

TARGET = 'course_buoy_a_yellow_1'


def read_spawn(model, qpos, buoys):
    """Validate before mutation; return an assembly translation [m], or None."""
    filename = os.environ.get('UUV_DEMO_SPAWN_REQUEST')
    if not filename:
        return None
    path = Path(filename)
    spec = json.loads(path.read_text())
    if set(spec) != {'seed', 'x_m', 'y_m'} or type(spec['seed']) is not int or not 0 <= spec['seed'] < 2**63:
        raise ValueError('Invalid buoy spawn request')
    for key, low, high in [('x_m', -2., 2.), ('y_m', -1.5, 1.5)]:
        value = spec[key]
        if (not isinstance(value, list) or len(value) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)
                or not low <= value[0] < value[1] <= high):
            raise ValueError('Spawn bounds outside the central research pool: '+key)
    buoy = next(b for b in buoys if b.name == TARGET)
    anchor = model.body(TARGET+'_magnet_base').id
    if buoy.free_qposadr < 0 or model.body_parentid[anchor] != 0:
        raise ValueError('Unsupported target anchor/free-joint topology')
    rng = random.Random(spec['seed'])
    for _ in range(100):
        xy = [rng.uniform(*spec['x_m']), rng.uniform(*spec['y_m'])]
        # The research-pool vehicle free joint is first; require clearance [m].
        if math.hypot(xy[0]-qpos[0], xy[1]-qpos[1]) >= 1.25:
            break
    else:
        raise ValueError('No collision-clear spawn sampled')
    delta = [xy[i]-qpos[buoy.free_qposadr+i] for i in range(2)]
    return dict(spec=spec, xy_m=xy, delta=delta, anchor=anchor,
                qposadr=buoy.free_qposadr, body=buoy.body_id,
                result_path=path.with_name('spawn-result.json'))


def apply_spawn(model, data, placement):
    """Translate the anchor/rope subtree and free float by the same XY [m]."""
    if placement is None:
        return
    delta = placement['delta']
    model.body_pos[placement['anchor'], :2] += delta
    model.body_pos[placement['body'], :2] += delta
    address = placement['qposadr']
    data.qpos[address:address+2] += delta


def record_spawn(placement):
    if placement is not None:
        path = placement['result_path']
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(schema='uuv.spawn.v1', target=TARGET,
                                             **placement['spec'], xy_m=placement['xy_m'])))
        temporary.replace(path)

"""Read saved rollouts without loading policies, starting services, or training."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re

from release_validation import validate_release
from schedule import round_plan


def audit_run(run):
    """Return provenance and descriptive diagnostics, not a success-rate trial."""
    config = json.loads((run / 'config.json').read_text())
    episodes = []
    roles = {}
    for path in sorted(run.glob('env-*/outputs/rollout-*/episode.json')):
        raw = path.read_bytes()
        episode = json.loads(raw)
        trace = episode['trace']
        # run.py uses zero-based rollout directories but one-based collection
        # checkpoints/status waves. Keep both identifiers explicit.
        wave = int(re.fullmatch(r'rollout-(\d+)(?:-.*)?', path.parent.name)[1])
        collection = wave + 1
        role, repeat = round_plan(wave, config.get('evaluation_repeats', 1))
        terms = Counter()
        failures = Counter()
        qualified = 0
        near = 0
        max_dwell = 0.
        for step in trace:
            terms.update(step['terms'])
            qualified += bool(step.get('progress_qualified'))
            max_dwell = max(max_dwell, step.get('dwell_s', 0.))
            m = step['measurement']
            if m['detached'] or 'slot_offset_m' not in m:
                continue
            x, y, z = m['slot_offset_m']
            if math.sqrt(x*x+y*y+z*z) > 1.:
                continue
            near += 1
            conditions = {
                'sample_gap': 0 < step['duration'] <= .25,
                'entry_x': -.02 <= x <= 1., 'lateral': abs(y) <= .05,
                'height': abs(z) <= .05,
                'bearing': abs(math.atan2(y, max(x, .08))) <= .20,
                'relative_speed': step.get('relative_speed_m_s', math.inf) <= .08,
                'yaw_rate': abs(m['yaw_rate_rad_s']) <= .15,
            }
            failures.update(key for key, ok in conditions.items() if not ok)
        validation = validate_release(trace[-1]['measurement'], released=episode['released']) if trace else None
        row = dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest(),
                   role=role, repeat=repeat, collection=collection,
                   release_tier=episode['release_tier'], steps=len(trace),
                   qualified_steps=qualified, max_dwell_s=max_dwell,
                   attached_near_steps=near, gate_failures_near=dict(failures),
                   reward_terms=dict(terms), validation=validation)
        episodes.append(row)
        summary = roles.setdefault(role, dict(episodes=0, steps=0, qualified_steps=0,
                                            release_tiers=Counter(), reward_terms=Counter(),
                                            attached_near_steps=0, gate_failures_near=Counter()))
        summary['episodes'] += 1
        summary['steps'] += len(trace)
        summary['qualified_steps'] += qualified
        summary['release_tiers'][episode['release_tier']] += 1
        summary['reward_terms'].update(terms)
        summary['attached_near_steps'] += near
        summary['gate_failures_near'].update(failures)
    return dict(schema='uuv.precision_audit.v1', run=str(run.resolve()),
                reward_version=config.get('reward_version'),
                warning='Descriptive saved episodes; mixed checkpoints and repeated layouts. '
                        'Partial evaluation rounds are included only as diagnostics. '
                        'Missing evidence is unknown, not a measured collision or stability failure. '
                        'Gate failures overlap; speed checks apply to precision_v4.',
                roles=roles, episodes=episodes)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit_run(args.run)
    # Exclusive creation prevents accidental overwriting of previous evidence.
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(report['roles'], indent=2))

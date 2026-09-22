"""Validate paired images and the existing 23-state/4-action recording contract."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def validate(root):
    rows = []
    for episode in sorted(root.glob('episode_*')):
        manifest = json.loads((episode / 'manifest.json').read_text())
        with np.load(episode / 'samples.npz') as samples:
            n = manifest['frames']
            assert n > 0, episode
            assert samples['observation_state'].shape == (n, 23), episode
            assert samples['action'].shape == (n, 4), episode
            assert np.isfinite(samples['observation_state']).all(), episode
            assert np.isfinite(samples['action']).all(), episode
            assert np.all(np.diff(samples['ros_timestamp']) > 0), episode
            assert np.allclose(samples['action'], (samples['rc_pwm'] - 1500.) / 400.), episode
            nonneutral = int(np.count_nonzero(np.any(abs(samples['action']) > .01, axis=1)))
        sizes = set()
        for camera in ('ego', 'buoy_release'):
            images = sorted((episode / 'frames' / camera).glob('*.jpg'))
            assert len(images) == n, (episode, camera, len(images), n)
            for path in images:
                image = cv2.imread(str(path))
                assert image is not None, path
                sizes.add(tuple(image.shape[:2]))
        assert manifest['provenance']['data_source'] == 'simulation', episode
        evidence = json.loads((episode / 'auto_collection.json').read_text())
        assert evidence['success'] == manifest['success'], episode
        rows.append(dict(episode=episode.name, frames=n, images=2*n,
                         success=manifest['success'], nonneutral_frames=nonneutral,
                         image_sizes_hw=sorted(sizes), reason=evidence['reason']))
    assert rows, 'No recordings found'
    return rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = json.dumps(validate(args.root), indent=2)
    if args.output:
        args.output.write_text(result)
    print(result)

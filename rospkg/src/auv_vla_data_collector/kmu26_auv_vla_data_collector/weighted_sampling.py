"""Opt-in, evidence-bound sampling of visually reviewed interaction windows.

Annotations change training frequency only: they are never policy observations or
CAP targets. Visual proximity does not certify physical contact or detachment.
"""

import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Sampler


def chunk_weights(dataset, annotation_path: Path):
    """Return bounded weights for complete chunks in the annotated training split.

    Frame intervals are inclusive. Weight is the mean frame importance across
    the 16-step action horizon, preserving learning of the lead-in commands.
    Unreviewed frames retain weight one; no terminal frames are synthesized.
    """
    config = json.loads(Path(annotation_path).read_text())
    root = Path(config['dataset_path']).resolve()
    if Path(dataset.dataset_path).resolve() != root:
        raise ValueError('Weight annotations belong to a different dataset')
    if config.get('schema_version') != 1 or config.get('split') != 'train':
        raise ValueError('Only schema-1 training annotations are supported')
    manifests = root / 'meta/source_manifests.jsonl'
    if hashlib.sha256(manifests.read_bytes()).hexdigest() != config['source_manifests_sha256']:
        raise ValueError('Source manifest digest changed')
    frame_weights = {}
    for episode in config['episodes']:
        index, length = episode['episode_index'], episode['frames']
        if index in frame_weights:
            raise ValueError('Duplicate annotated episode')
        parquet = root / 'data/chunk-000' / f'episode_{index:06d}.parquet'
        if hashlib.sha256(parquet.read_bytes()).hexdigest() != episode['parquet_sha256']:
            raise ValueError('Annotated episode data changed')
        weights = np.ones(length, dtype=np.float64)
        for window in episode['windows']:
            start, end, weight = window['start_frame'], window['end_frame'], window['weight']
            if not (isinstance(start, int) and isinstance(end, int) and 0 <= start <= end < length):
                raise ValueError('Invalid annotation frame interval')
            if not np.isfinite(weight) or not 1 <= weight <= 3:
                raise ValueError('Visual weights must be finite and bounded to [1, 3]')
            if window.get('evidence') not in ('visual_proximity_review', 'visual_cue_heuristic') or not window.get('review_image'):
                raise ValueError('Explicit visual evidence required')
            weights[start:end + 1] = np.maximum(weights[start:end + 1], weight)
        frame_weights[index] = weights
    lengths = dict(zip(map(int, dataset.trajectory_ids), map(int, dataset.trajectory_lengths)))
    if set(lengths) != set(frame_weights) or any(len(frame_weights[i]) != n for i, n in lengths.items()):
        raise ValueError('Annotation episode lengths do not match dataset')
    result = []
    for episode, start in dataset.all_steps:
        values = frame_weights[int(episode)][int(start):int(start) + 16]
        if len(values) != 16:
            raise ValueError('Incomplete action chunk cannot be weighted')
        result.append(float(values.mean()))
    return np.asarray(result, dtype=np.float64)


class ReviewedWindowSampler(Sampler):
    """Seeded weighted draws with replacement, compatible with trainer epochs."""

    def __init__(self, dataset, weights, seed=0):
        self.dataset = dataset
        self.weights = torch.as_tensor(weights, dtype=torch.double)
        if len(self.weights) != len(dataset) or not torch.isfinite(self.weights).all() or (self.weights <= 0).any():
            raise ValueError('Invalid sampling weights')
        self.seed = seed
        self.epoch = 0

    def __iter__(self):
        generator = torch.Generator().manual_seed(self.seed + self.epoch)
        return iter(torch.multinomial(self.weights, len(self), replacement=True, generator=generator).tolist())

    def __len__(self):
        return len(self.dataset)

    def set_epoch(self, epoch):
        self.epoch = epoch
        if hasattr(self.dataset, 'set_epoch'):
            self.dataset.set_epoch(epoch)

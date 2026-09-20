"""Evidence binding and sampling semantics, independent of U0 model loading."""
import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('torch')
from kmu26_auv_vla_data_collector.weighted_sampling import chunk_weights, ReviewedWindowSampler


def fixture(tmp_path):
    (tmp_path/'meta').mkdir()
    (tmp_path/'data/chunk-000').mkdir(parents=True)
    (tmp_path/'meta/source_manifests.jsonl').write_bytes(b'manifest')
    (tmp_path/'data/chunk-000/episode_000000.parquet').write_bytes(b'data')
    c={'schema_version':1,'split':'train','dataset_path':str(tmp_path),'source_manifests_sha256':hashlib.sha256(b'manifest').hexdigest(),'episodes':[{'episode_index':0,'frames':32,'parquet_sha256':hashlib.sha256(b'data').hexdigest(),'windows':[{'start_frame':16,'end_frame':31,'weight':3,'evidence':'visual_proximity_review','review_image':'review.jpg'}]}]}
    p=tmp_path/'annotations.json';p.write_text(json.dumps(c))
    d=SimpleNamespace(dataset_path=tmp_path,trajectory_ids=[0],trajectory_lengths=[32],all_steps=[(0,i) for i in range(17)])
    return d,p,c


def test_horizon_overlap_and_uniform_lead_in(tmp_path):
    d,p,_=fixture(tmp_path)
    w=chunk_weights(d,p)
    np.testing.assert_allclose(w,np.linspace(1,3,17))


@pytest.mark.parametrize('bad', ['digest','split','range','weight','length','dataset'])
def test_reject_invalid_annotations(tmp_path,bad):
    d,p,c=fixture(tmp_path)
    if bad=='digest':c['source_manifests_sha256']='wrong'
    if bad=='split':c['split']='validation'
    if bad=='range':c['episodes'][0]['windows'][0]['end_frame']=32
    if bad=='weight':c['episodes'][0]['windows'][0]['weight']=float('nan')
    if bad=='length':c['episodes'][0]['frames']=33
    if bad=='dataset':d.dataset_path=tmp_path/'other'
    p.write_text(json.dumps(c))
    with pytest.raises(ValueError):chunk_weights(d,p)


def test_seed_epoch_and_weighted_frequency():
    d=list(range(10000));s=ReviewedWindowSampler(d,[1]*5000+[3]*5000,seed=42)
    first=list(s);assert first==list(s)
    assert .72 < np.mean(np.array(first)>=5000) < .78
    s.set_epoch(1);assert list(s)!=first

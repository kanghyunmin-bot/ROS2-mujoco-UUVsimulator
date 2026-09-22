"""Atomic local-file messages; only read files from this run's private directory."""
import pickle
import json
from pathlib import Path


def write(path, value):
    path=Path(path);temp=path.with_suffix('.tmp')
    with temp.open('wb') as f:pickle.dump(value,f,protocol=4)
    temp.replace(path)


def read(path):
    with Path(path).open('rb') as f:return pickle.load(f)


def status(path, value):
    path=Path(path);temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2));temp.replace(path)

"""Download the optional release model, verify hashes, and install without tar traversal."""
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
version=dict(line.split('=',1) for line in (ROOT/'release/versions.env').read_text().splitlines())['RELEASE_VERSION']
url=f'https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator/releases/download/v{version}/'
cache=ROOT/'.cache/release-model';cache.mkdir(parents=True,exist_ok=True)
with urllib.request.urlopen(url+'model-download.json',timeout=60) as r:manifest=json.load(r)
assert manifest['version']==version and manifest['folder']=='fork-medium-10000'
dest=ROOT/'models'/manifest['folder']
if dest.exists():raise SystemExit(f'Already installed: {dest}; existing files preserved')
if shutil.disk_usage(ROOT).free<25*2**30:raise SystemExit('Need 25 GiB free for download and extraction')
archive=cache/'model.tar.gz'
with archive.open('wb') as output:
    for part in manifest['parts']:
        name=part['name']
        assert '/' not in name and name.startswith(f'uuv-fork-medium-10000-{version}.tar.gz.part')
        h=hashlib.sha256();count=0
        print('Downloading',name,flush=True)
        with urllib.request.urlopen(url+name,timeout=120) as response:
            while data:=response.read(8*1024*1024):output.write(data);h.update(data);count+=len(data)
        if count!=part['size'] or h.hexdigest()!=part['sha256']:
            raise SystemExit(f'Checksum mismatch: {name}; model not installed')
staging=cache/'extracted';staging.mkdir(exist_ok=True)
with tarfile.open(archive) as source:
    for member in source:
        target=staging/member.name
        if not target.resolve().is_relative_to(staging.resolve()) or not member.name.startswith('fork-medium-10000/') and member.name!='fork-medium-10000':
            raise SystemExit('Unsafe model archive path')
        if member.isdir():target.mkdir(parents=True,exist_ok=True)
        elif member.isfile():
            target.parent.mkdir(parents=True,exist_ok=True)
            with source.extractfile(member) as src,target.open('wb') as dst:shutil.copyfileobj(src,dst)
        else:raise SystemExit('Unsupported archive entry')
# The standard installer checks the deployment contract and shard completeness.
import subprocess,sys
subprocess.run([sys.executable,str(ROOT/'release/install_model.py'),str(staging/manifest['folder'])],check=True)
print('Model installed. Restart ./release.sh web to refresh the model list.')

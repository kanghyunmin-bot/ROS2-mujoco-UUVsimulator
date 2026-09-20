"""Build self-contained source and inference-only model assets from the release checkout."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--version',default='2026.09.20.1');p.add_argument('--model',action='store_true');a=p.parse_args()
OUT=ROOT/'release_artifacts';OUT.mkdir(exist_ok=True)
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()
if a.model:
    folder=OUT/'model-staging/fork-medium-10000'
    tar=OUT/f'uuv-fork-medium-10000-{a.version}.tar.gz'
    with tarfile.open(tar,'w:gz',compresslevel=1) as archive:archive.add(folder,arcname=folder.name)
    parts=[]
    with tar.open('rb') as stream:
        i=0
        while True:
            chunk=stream.read(1800*1024*1024)
            if not chunk:break
            part=OUT/(tar.name+f'.part{i:02d}');part.write_bytes(chunk)
            parts.append(dict(name=part.name,size=part.stat().st_size,sha256=sha(part)));i+=1
    (OUT/'model-download.json').write_text(json.dumps(dict(version=a.version,folder=folder.name,parts=parts),indent=2))
else:
    paths=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().split('\0')
    source=OUT/f'uuv-sim-{a.version}-source.tar.gz'
    with tarfile.open(source,'w:gz',compresslevel=6) as archive:
        for name in sorted(filter(None,paths)):
            path=ROOT/name
            if not path.is_file() and not path.is_symlink():continue
            if path.is_file() and path.suffix=='.pt' and path.stat().st_size<1024:
                raise RuntimeError(f'Git LFS object not hydrated: {name}')
            archive.add(path,arcname=f'uuv-sim-{a.version}/{name}',recursive=False)
    print(source)
files=sorted(p for p in OUT.iterdir() if p.is_file() and (p.name.endswith('-source.tar.gz') or '.part' in p.name or p.name in ['model-download.json','VALIDATION.md','RELEASE_NOTES.md']))
(OUT/'SHA256SUMS').write_text(''.join(f'{sha(p)}  {p.name}\n' for p in files))
print('Checksums written for',len(files),'assets')

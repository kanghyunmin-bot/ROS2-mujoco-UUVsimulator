"""Isolate recordings by camera configuration and optional control profile."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def recording_partition(
    app_root: Path,
    scene: Path,
    camera: dict,
    *,
    control_profile: str | None = None,
) -> tuple[Path, dict]:
    root = ET.parse(scene).getroot()
    geometry = {}
    for name in ('stereo_left', 'stereo_right'):
        item = root.find(f'.//camera[@name="{name}"]')
        if item is None:
            raise ValueError(f'Missing recording camera: {name}')
        geometry[name] = {
            key: [round(float(value), 9) for value in item.get(key).split()]
            for key in ('pos', 'quat', 'xyaxes', 'fovy', 'focal', 'sensorsize', 'resolution')
            if item.get(key) is not None
        }
    profile = dict(cameras=geometry, width=int(camera.get('width', 640)),
                   height=int(camera.get('height', 360)),
                   optics_profile=str(camera.get('optics_profile', 'inherited')))
    if control_profile is not None:
        profile['control_profile'] = control_profile
    fingerprint = hashlib.sha256(json.dumps(profile, sort_keys=True).encode()).hexdigest()
    profile['id'] = 'camera-' + fingerprint[:12]
    profile['sha256'] = fingerprint
    return app_root / 'outputs/vla-experiments' / profile['id'] / 'datasets', profile

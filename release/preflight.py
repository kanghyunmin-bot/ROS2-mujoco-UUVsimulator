"""Check host prerequisites without altering drivers, Docker, or user groups."""
import os
import platform
import shutil
import subprocess
from pathlib import Path

values = {}
for line in Path('/etc/os-release').read_text().splitlines():
    if '=' in line:
        key, value = line.split('=', 1)
        values[key] = value.strip('"')
if values.get('ID') != 'ubuntu' or values.get('VERSION_ID') not in ('22.04', '24.04'):
    raise SystemExit('Supported host: Ubuntu 22.04 or 24.04 (use the documented container route).')
if platform.machine() != 'x86_64':
    raise SystemExit('This release targets x86_64, not Jetson/ARM.')
if os.getuid() == 0:
    raise SystemExit('Run as your ordinary account with Docker access, not sudo/root.')
for command in ('docker', 'git', 'python3'):
    if not shutil.which(command):
        raise SystemExit(f'Missing {command}; see docs/RELEASE_INSTALL.md')
subprocess.run(['docker', 'info'], check=True, stdout=subprocess.DEVNULL)
subprocess.run(['docker', 'compose', 'version'], check=True)
free = shutil.disk_usage(Path(__file__).resolve().parents[1]).free / 2**30
if free < 25:
    raise SystemExit(f'At least 25 GiB free needed for installation; available {free:.1f} GiB.')
print(f'Host {values["VERSION_ID"]} x86_64: Docker access OK; free {free:.1f} GiB')

"""Validate dependencies and the operations used by U0 on the installed GPU."""
import subprocess
import sys
import tempfile
from pathlib import Path

result = subprocess.run([sys.executable, '-m', 'pip', 'check'], text=True, capture_output=True)
# Decord's py3 wheel incorrectly embeds a cp36 tag in WHEEL. Verify decoding
# below; do not hide missing dependencies or any other compatibility report.
known = 'decord 0.6.0 is not supported on this platform'
errors = [line for line in result.stdout.splitlines() if line != known]
if result.returncode and errors:
    raise SystemExit('\n'.join(errors) + result.stderr)
import cv2
import decord
import numpy as np
import torch
from pytorch3d.transforms import quaternion_to_matrix
from flash_attn import flash_attn_func
assert torch.cuda.is_available(), 'CUDA GPU unavailable'
assert quaternion_to_matrix(torch.tensor([[1., 0., 0., 0.]])).shape == (1, 3, 3)
x = torch.randn(1, 16, 2, 64, device='cuda', dtype=torch.float16)
assert torch.isfinite(flash_attn_func(x, x, x)).all()
with tempfile.TemporaryDirectory() as folder:
    path = str(Path(folder) / 'decode.mp4')
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), 10, (64, 64))
    assert writer.isOpened()
    for i in range(3):
        writer.write(np.full((64, 64, 3), i * 60, dtype=np.uint8))
    writer.release()
    frames = decord.VideoReader(path).get_batch([0, 1, 2]).asnumpy()
    assert frames.shape == (3, 64, 64, 3)
print('VLA dependency, video decoding, transforms and CUDA attention checks passed:', torch.cuda.get_device_name(0))

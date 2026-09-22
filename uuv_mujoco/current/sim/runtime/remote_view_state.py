"""Optional latest-frame export; a slow browser never waits inside physics."""
import os
from pathlib import Path
import queue
import threading
import time
import uuid

import numpy as np


class RemoteViewState:
    def __init__(self, mujoco, model, directory, fps=15):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.generation = uuid.uuid4().hex
        self.interval = 1.0 / fps
        self.next_frame = 0.0
        self.failed = False
        self.frames = queue.Queue(maxsize=1)
        temporary = self.directory / 'model.tmp.mjb'
        mujoco.mj_saveModel(model, str(temporary), None)
        os.replace(temporary, self.directory / 'model.mjb')
        threading.Thread(target=self._write, daemon=True).start()

    def publish(self, data):
        now = time.monotonic()
        if self.failed or now < self.next_frame:
            return
        self.next_frame = now + self.interval
        frame = dict(generation=np.array(self.generation), wall_time=np.array(time.time()),
                     sim_time=np.array(data.time), xpos=data.xpos.copy(),
                     xmat=data.xmat.copy(), mocap_pos=data.mocap_pos.copy(),
                     mocap_quat=data.mocap_quat.copy())
        try:
            self.frames.get_nowait()
        except queue.Empty:
            pass
        self.frames.put_nowait(frame)

    def _write(self):
        try:
            while True:
                frame = self.frames.get()
                with (self.directory / 'frame.tmp').open('wb') as f:
                    np.savez(f, **frame)
                os.replace(self.directory / 'frame.tmp', self.directory / 'frame.npz')
        except Exception as error:
            self.failed = True
            print(f'[remote-view] export disabled: {error}', flush=True)

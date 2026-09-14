"""Animate a non-colliding pool surface only at viewer refresh cadence."""

import threading

import numpy as np

from bridge.water_lighting import wave_field


class WaterSurfaceVisual:
    """Update a shared visual height field; never modify physical water level."""

    def __init__(self, mujoco, model) -> None:
        self.field_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_HFIELD, "water_ripple_field"
        )
        self.model = model
        self._upload_pending = False
        if self.field_id < 0:
            return
        surface = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "water_surface")
        ripple = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "water_ripples")
        model.hfield_size[self.field_id, :2] = model.geom_size[surface, :2]
        model.geom_pos[ripple, :2] = model.geom_pos[surface, :2]
        model.geom_pos[ripple, 2] = model.geom_pos[surface, 2] - 0.024
        size = model.hfield_size[self.field_id]
        rows, cols = model.hfield_nrow[self.field_id], model.hfield_ncol[self.field_id]
        self.x = (
            np.linspace(-size[0], size[0], cols, dtype=np.float32)[None, :]
            + model.geom_pos[surface, 0]
        )
        self.y = (
            np.linspace(-size[1], size[1], rows, dtype=np.float32)[:, None]
            + model.geom_pos[surface, 1]
        )
        start = model.hfield_adr[self.field_id]
        self.values = model.hfield_data[start : start + rows * cols].reshape(rows, cols)

    def update(self, time_s: float) -> None:
        """Set display elevations from simulation time [s]."""
        if self.field_id >= 0:
            height, _, _ = wave_field(self.x, self.y, time_s)
            self.values[:] = 0.5 + height / 0.04

    def update_viewer(self, viewer, time_s: float) -> None:
        """Skip busy uploads so the UI cannot block the physics/control thread."""
        if self.field_id < 0 or self._upload_pending:
            return
        self._upload_pending = True
        with viewer.lock():
            self.update(time_s)

        def upload():
            try:
                if viewer.is_running():
                    viewer.update_hfield(self.field_id)
            except RuntimeError:
                # Closing the native viewer may invalidate an outstanding upload.
                if viewer.is_running():
                    raise
            finally:
                self._upload_pending = False

        threading.Thread(target=upload, name="uuv-water-upload", daemon=True).start()

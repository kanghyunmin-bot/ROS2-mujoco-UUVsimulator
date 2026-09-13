"""Scene contact limits must survive GUI overrides and FCU alignment."""

from pathlib import Path
import sys

import mujoco
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim.runtime.model_runtime_setup import _align_fcu_timestep, _apply_timestep_override


@pytest.mark.parametrize("requested", [.0005, .0025, .005])
def test_submillisecond_scene_limit_survives_gui_request(requested):
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <option timestep=".0005"/>
      <custom><numeric name="buoy_contact_max_timestep" data=".0005"/></custom>
    </mujoco>''')
    _apply_timestep_override(
        model, mujoco_module=mujoco,
        env_float=lambda name, default: requested if name == "UUV_MUJOCO_TIMESTEP" else default,
        env_flag=lambda name, default: default,
    )
    _align_fcu_timestep(model, 400.)
    assert model.opt.timestep <= .0005
    assert .0025 / model.opt.timestep == pytest.approx(round(.0025 / model.opt.timestep))

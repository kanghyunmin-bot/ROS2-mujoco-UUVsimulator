"""Opt-in KMU26 transfer config; use the same config for training and inference.

Meters, acceleration, commands and validity are not periodic angles. Preserve the
23 physical state values instead of applying sin/cos to every field (46 values).
Existing upstream-trained checkpoints must keep their original preprocessing.
"""

from gr00t.experiment.data_config import Kmu26AuvRealDataConfig
from gr00t.data.transform.state_action import StateActionSinCosTransform


class Kmu26TransferDataConfig(Kmu26AuvRealDataConfig):
    def transform(self):
        result = super().transform()
        result.transforms = [
            t
            for t in result.transforms
            if not isinstance(t, StateActionSinCosTransform)
        ]
        return result

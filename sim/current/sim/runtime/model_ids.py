"""Named MuJoCo object and sensor lookup helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Mapping


def named_object_ids(model: Any, mujoco_module: Any, obj_type: Any, names: Iterable[str]) -> dict[str, int]:
    """Return object ids for names that exist in the model."""
    ids: dict[str, int] = {}
    for name in names:
        oid = int(mujoco_module.mj_name2id(model, obj_type, name))
        if oid >= 0:
            ids[str(name)] = oid
    return ids


def keyed_object_ids(model: Any, mujoco_module: Any, obj_type: Any, names_by_key: Mapping[str, str]) -> dict[str, int]:
    """Return object ids keyed by logical names instead of MuJoCo object names."""
    ids: dict[str, int] = {}
    for key, object_name in names_by_key.items():
        oid = int(mujoco_module.mj_name2id(model, obj_type, object_name))
        if oid >= 0:
            ids[str(key)] = oid
    return ids


@dataclass(frozen=True)
class SensorDataReader:
    """Read named sensor data slices from a MuJoCo data object."""

    model: Any
    data: Any
    sensor_ids: Mapping[str, int]

    def value(self, name: str):
        """Return a copy of the named sensor's current data slice."""
        sid = int(self.sensor_ids.get(name, -1))
        if sid < 0:
            return None
        adr = int(self.model.sensor_adr[sid])
        dim = int(self.model.sensor_dim[sid])
        return self.data.sensordata[adr : adr + dim].copy()


__all__ = ["SensorDataReader", "keyed_object_ids", "named_object_ids"]

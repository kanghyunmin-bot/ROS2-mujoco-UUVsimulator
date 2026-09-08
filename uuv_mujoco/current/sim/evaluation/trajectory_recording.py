"""ROS-independent buffering for evaluation trajectory recording."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class PoseRecord:
    """One TUM pose record."""

    timestamp_s: float
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


class PoseRecordBuffer:
    """Collect strictly monotonic poses without hiding timestamp faults."""

    def __init__(
        self,
        label: str,
        *,
        expected_frame_id: str | None = None,
        expected_child_frame_id: str | None = None,
    ) -> None:
        self.label = str(label)
        self.expected_frame_id = (
            None if expected_frame_id is None else str(expected_frame_id)
        )
        self.expected_child_frame_id = (
            None if expected_child_frame_id is None else str(expected_child_frame_id)
        )
        self.records: list[PoseRecord] = []
        self.rejected_nonmonotonic = 0
        self.rejected_invalid = 0
        self.rejected_frame_contract = 0
        self.observed_frame_ids: set[str] = set()
        self.observed_child_frame_ids: set[str] = set()

    def append(
        self,
        timestamp_s: float,
        position_m,
        quaternion_xyzw,
        *,
        frame_id: str | None = None,
        child_frame_id: str | None = None,
    ) -> bool:
        """Validate and append one pose, returning whether it was accepted."""

        actual_frame_id = None if frame_id is None else str(frame_id)
        actual_child_frame_id = None if child_frame_id is None else str(child_frame_id)
        if actual_frame_id is not None:
            self.observed_frame_ids.add(actual_frame_id)
        if actual_child_frame_id is not None:
            self.observed_child_frame_ids.add(actual_child_frame_id)
        if (
            self.expected_frame_id is not None
            and actual_frame_id != self.expected_frame_id
        ) or (
            self.expected_child_frame_id is not None
            and actual_child_frame_id != self.expected_child_frame_id
        ):
            self.rejected_frame_contract += 1
            return False

        timestamp = float(timestamp_s)
        position = np.asarray(position_m, dtype=np.float64)
        quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
        if (
            not np.isfinite(timestamp)
            or timestamp < 0.0
            or position.shape != (3,)
            or quaternion.shape != (4,)
            or not np.all(np.isfinite(position))
            or not np.all(np.isfinite(quaternion))
        ):
            self.rejected_invalid += 1
            return False
        quaternion_norm = float(np.linalg.norm(quaternion))
        if quaternion_norm <= 1.0e-12:
            self.rejected_invalid += 1
            return False
        if self.records and timestamp <= self.records[-1].timestamp_s:
            self.rejected_nonmonotonic += 1
            return False
        quaternion /= quaternion_norm
        self.records.append(
            PoseRecord(
                timestamp,
                tuple(float(value) for value in position),
                tuple(float(value) for value in quaternion),
            )
        )
        return True

    def write_tum(self, path: str | Path) -> None:
        """Write accepted records in TUM trajectory format."""

        lines = [
            " ".join(
                (
                    f"{record.timestamp_s:.9f}",
                    *(f"{value:.9f}" for value in record.position_m),
                    *(f"{value:.9f}" for value in record.quaternion_xyzw),
                )
            )
            for record in self.records
        ]
        Path(path).write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    def summary(self) -> dict[str, object]:
        """Return JSON-serializable timestamp-health metadata."""

        return {
            "label": self.label,
            "accepted_count": len(self.records),
            "rejected_nonmonotonic_count": self.rejected_nonmonotonic,
            "rejected_invalid_count": self.rejected_invalid,
            "rejected_frame_contract_count": self.rejected_frame_contract,
            "expected_frame_id": self.expected_frame_id,
            "expected_child_frame_id": self.expected_child_frame_id,
            "observed_frame_ids": sorted(self.observed_frame_ids),
            "observed_child_frame_ids": sorted(self.observed_child_frame_ids),
            "start_time_s": self.records[0].timestamp_s if self.records else None,
            "end_time_s": self.records[-1].timestamp_s if self.records else None,
        }


__all__ = ["PoseRecord", "PoseRecordBuffer"]

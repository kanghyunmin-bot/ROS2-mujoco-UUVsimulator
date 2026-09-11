"""Optional U0 loader boundary; run in the U0 environment, not the ROS node.

Use this dataset class in place of LeRobotSingleDataset for KMU26 training.
Only complete future action chunks are indexed. No terminal action is invented.
"""

import json
from pathlib import Path

import numpy as np
import pandas as pd
from gr00t.data.dataset import LeRobotSingleDataset


def validate_demonstrations(dataset_path: Path) -> None:
    """Reject connection checks, unknown provenance and interrupted recordings."""
    path = dataset_path / "meta" / "source_manifests.jsonl"
    if not path.is_file():
        raise ValueError("Missing source manifests; re-export with provenance")
    for line in path.read_text().splitlines():
        m = json.loads(line)
        p = m.get("provenance", {})
        if (
            p.get("collection_kind") != "task_demonstration"
            or p.get("data_source") not in ("real", "simulation")
            or not p.get("session_id")
            or not p.get("context")
            or m.get("termination_reason") != "operator_stop"
        ):
            raise ValueError(
                "Not a reviewed task demonstration: check provenance/termination"
            )
        if not m.get("success"):
            raise ValueError(
                "Recovery episodes require a separate reviewed training selection"
            )
    manifests = [json.loads(line) for line in path.read_text().splitlines()]
    for i, manifest in enumerate(manifests):
        log = (
            dataset_path
            / "meta"
            / "acquisition"
            / f"episode_{i:06d}"
            / "vehicle_state.jsonl"
        )
        if not log.is_file():
            raise ValueError("Missing per-frame vehicle/control telemetry")
        states = [json.loads(line) for line in log.read_text().splitlines()]
        if len(states) != manifest["frames"] or any(
            not s["connected"]
            or not s["armed"]
            or s["mode"] != manifest["provenance"]["expected_mode"]
            or s["rc_publishers"] != 1
            or s["state_receipt_age_wall_s"] > 2.0
            for s in states
        ):
            raise ValueError("Vehicle/control state changed or was unverified")
        table = pd.read_parquet(
            dataset_path / "data" / "chunk-000" / f"episode_{i:06d}.parquet"
        )
        state = np.stack(table["observation.state"])
        stamps = np.stack(table["telemetry.source_timestamp"])
        if (
            state.shape != (manifest["frames"], 23)
            or stamps.shape != (manifest["frames"], 7)
            or not np.isfinite(stamps).all()
            or not np.isfinite(state).all()
            or not np.all(state[:, 19:21] == 1)
            or np.any(np.diff(stamps[:, :2], axis=0) <= 0)
        ):
            raise ValueError("Stale, duplicate or invalid training camera/state")
    # Requested RC remains the behavior-cloning target. FCU telemetry is retained
    # for latency/acceptance review, not relabeled as instantaneous measured thrust.


def assert_disjoint_sessions(training_path: Path, validation_path: Path) -> None:
    """Keep all demonstrations from one collection session in the same split."""

    def sessions(root):
        rows = (root / "meta" / "source_manifests.jsonl").read_text().splitlines()
        ids = {
            json.loads(row).get("provenance", {}).get("session_id", "") for row in rows
        }
        if not ids or "" in ids:
            raise ValueError("Session IDs required to verify split leakage")
        return ids

    if sessions(training_path) & sessions(validation_path):
        raise ValueError("Training/validation sessions overlap")


class Kmu26TrainingDataset(LeRobotSingleDataset):
    """U0 dataset indexing complete 16-step chunks, with an explicit inspection mode."""

    def __init__(self, *args, inspection: bool = False, **kwargs):
        dataset_path = Path(kwargs.get("dataset_path", args[0] if args else "."))
        if not inspection:
            validate_demonstrations(dataset_path)
        self.inspection = inspection
        super().__init__(*args, **kwargs)
        if not np.isclose(float(self._lerobot_info_meta["fps"]), 10.0):
            raise ValueError("KMU26 policy requires 10 Hz data")
        if not self.all_steps:
            raise ValueError("No complete action chunks (at least 16 frames required)")

    def _get_all_steps(self):
        action = self.modality_configs["action"]
        if action.delta_indices != list(range(16)):
            raise ValueError("KMU26 requires future action indices 0..15")
        return [
            (trajectory_id, base_index)
            for trajectory_id, length in zip(
                self.trajectory_ids, self.trajectory_lengths
            )
            for base_index in range(max(0, int(length) - 15))
        ]

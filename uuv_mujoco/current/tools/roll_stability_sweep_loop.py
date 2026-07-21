"""Candidate execution loop for roll stability sweeps."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from roll_stability_candidates import Candidate
from roll_stability_file_edits import apply_candidate
from roll_stability_runner import run_candidate, stop_stack, write_summary
from roll_stability_sweep_files import OriginalFileTexts, restore_original_files
from roll_stability_sweep_paths import SweepPaths


def print_candidate_result(candidate: Candidate, result: dict[str, Any]) -> None:
    if "score" in result:
        print(
            "[sweep] result "
            f"{candidate.name}: score={result['score']:.3f}, "
            f"roll_rms={result['roll_rms_deg']:.2f}deg, "
            f"gyro_x_rms={result['gyro_x_rms_rad_s']:.3f}rad/s, "
            f"depth_std={result['depth_std_m']:.3f}m",
            flush=True,
        )
    else:
        print(f"[sweep] result {candidate.name}: FAIL {result.get('error')}", flush=True)


def run_sweep_candidates(
    *,
    candidates: list[Candidate],
    paths: SweepPaths,
    original_texts: OriginalFileTexts,
    out_dir: Path,
    settle_s: float,
    measure_s: float,
    stimulus: str,
    hold_mode: str,
    axis_command: float,
    pulse_s_override: float | None,
    wait_ready: bool,
) -> list[dict[str, Any]]:
    results: list[dict[str, Any]] = []
    try:
        for idx, candidate in enumerate(candidates, start=1):
            print(f"[sweep] {idx}/{len(candidates)} apply {candidate.name}: {candidate.note}", flush=True)
            restore_original_files(paths, original_texts)
            apply_candidate(
                profile_path=paths.profile_path,
                scene_path=paths.scene_path,
                mapping_path=paths.mapping_path,
                original_profile_text=original_texts.profile_text,
                original_scene_text=original_texts.scene_text,
                original_mapping_text=original_texts.mapping_text,
                candidate=candidate,
            )
            result = run_candidate(
                candidate=candidate,
                root=paths.root,
                start_script=paths.start_script,
                reset_script=paths.reset_script,
                out_dir=out_dir,
                settle_s=settle_s,
                measure_s=measure_s,
                stimulus=stimulus,
                hold_mode=hold_mode,
                axis_command=axis_command,
                pulse_s_override=pulse_s_override,
                wait_ready=wait_ready,
            )
            results.append(result)
            write_summary(out_dir, results)
            print_candidate_result(candidate, result)
    finally:
        restore_original_files(paths, original_texts)
        stop_stack(paths.root, paths.reset_script)
    return results


def print_final_result(results: list[dict[str, Any]]) -> int:
    ok_results = [result for result in results if "score" in result]
    if ok_results:
        best = min(ok_results, key=lambda item: float(item["score"]))
        print(f"[sweep] best={best['candidate']} score={best['score']:.3f}", flush=True)
    else:
        print("[sweep] no successful candidates", flush=True)
        return 1
    print("[sweep] original files restored; inspect summary.csv before applying anything.", flush=True)
    return 0


__all__ = ["print_final_result", "run_sweep_candidates"]

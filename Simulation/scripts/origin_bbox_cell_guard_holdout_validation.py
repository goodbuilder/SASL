#!/usr/bin/env python3
"""Validate origin-bbox cell guards under the Strict RDF all-axis gate.

Coordinate contract:
- Inputs are stereo-center camera RDF error fields, in meters.
- Output deltas are RMSE(raw_error_axis) - RMSE(corrected_error_axis).
- Position is Euclidean RMSE over Right/Down/Forward in the same RDF frame.
- No NED/ENU/PX4/body conversion is performed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


REPORTS = [
    "origin_bbox_reprojection_validation_attitude_aware_component_gains_longprofile_right_att_0p1_20260609_seed20260625",
    "origin_bbox_reprojection_validation_attitude_aware_component_gains_longprofile_right_att_0p1_20260609_seed20260626",
    "origin_bbox_reprojection_validation_attitude_aware_component_gains_right_att_0p1_20260609_seed20260625",
    "origin_bbox_reprojection_validation_attitude_aware_component_gains_ultralongprofile_right_att_0p1_20260609_seed20260626",
]


@dataclass(frozen=True)
class Params:
    cy_threshold: float
    cell_111_right_gain: float
    cell_111_down_gain: float
    cell_211_right_gain: float
    cell_211_down_gain: float


@dataclass
class Trial:
    report: str
    trial: int
    cell: np.ndarray
    left_cy: np.ndarray
    raw_right: np.ndarray
    raw_down: np.ndarray
    raw_forward: np.ndarray
    corrected_right: np.ndarray
    corrected_down: np.ndarray
    corrected_forward: np.ndarray


def rmse(values: np.ndarray) -> float:
    return math.sqrt(float(np.mean(values * values)))


def load_trials(root: Path) -> list[Trial]:
    trials: list[Trial] = []
    for report in REPORTS:
        report_dir = root / "Simulation" / "reports" / report
        for path in sorted(report_dir.glob("validation_trial_*_comparison.csv")):
            rows = list(csv.DictReader(path.open()))
            trial_id = int(path.name.split("_")[2])

            def arr(column: str) -> np.ndarray:
                return np.array([float(row[column]) for row in rows])

            trials.append(
                Trial(
                    report=report,
                    trial=trial_id,
                    cell=np.array(
                        [row["origin_bbox_reprojection_cell_key"] for row in rows]
                    ),
                    left_cy=arr("left_cy"),
                    raw_right=arr("raw_error_right_m"),
                    raw_down=arr("raw_error_down_m"),
                    raw_forward=arr("raw_error_forward_m"),
                    corrected_right=arr("origin_bbox_reprojection_error_right_m"),
                    corrected_down=arr("origin_bbox_reprojection_error_down_m"),
                    corrected_forward=arr("origin_bbox_reprojection_error_forward_m"),
                )
            )
    return trials


def apply_gain(raw: np.ndarray, corrected: np.ndarray, gain: float) -> np.ndarray:
    return raw + gain * (corrected - raw)


def evaluate_trial(trial: Trial, params: Params) -> dict[str, object]:
    right = trial.corrected_right.copy()
    down = trial.corrected_down.copy()
    forward = trial.corrected_forward

    mask_111 = (trial.cell == "1:1:1") & (trial.left_cy >= params.cy_threshold)
    right[mask_111] = apply_gain(
        trial.raw_right[mask_111],
        trial.corrected_right[mask_111],
        params.cell_111_right_gain,
    )
    down[mask_111] = apply_gain(
        trial.raw_down[mask_111],
        trial.corrected_down[mask_111],
        params.cell_111_down_gain,
    )

    mask_211 = trial.cell == "2:1:1"
    right[mask_211] = apply_gain(
        trial.raw_right[mask_211],
        trial.corrected_right[mask_211],
        params.cell_211_right_gain,
    )
    down[mask_211] = apply_gain(
        trial.raw_down[mask_211],
        trial.corrected_down[mask_211],
        params.cell_211_down_gain,
    )

    raw_right_rmse = rmse(trial.raw_right)
    raw_down_rmse = rmse(trial.raw_down)
    raw_forward_rmse = rmse(trial.raw_forward)
    corrected_right_rmse = rmse(right)
    corrected_down_rmse = rmse(down)
    corrected_forward_rmse = rmse(forward)

    raw_position_rmse = math.sqrt(
        raw_right_rmse**2 + raw_down_rmse**2 + raw_forward_rmse**2
    )
    corrected_position_rmse = math.sqrt(
        corrected_right_rmse**2
        + corrected_down_rmse**2
        + corrected_forward_rmse**2
    )
    deltas = {
        "right_delta_rmse_m": raw_right_rmse - corrected_right_rmse,
        "down_delta_rmse_m": raw_down_rmse - corrected_down_rmse,
        "forward_delta_rmse_m": raw_forward_rmse - corrected_forward_rmse,
        "position_delta_rmse_m": raw_position_rmse - corrected_position_rmse,
    }
    strict_pass = all(value > 0.0 for value in deltas.values())
    return {
        "report": trial.report,
        "trial": trial.trial,
        "sample_count": int(trial.raw_right.size),
        "cell_111_guard_count": int(mask_111.sum()),
        "cell_211_guard_count": int(mask_211.sum()),
        "pass_strict_rdf_all_axis": "yes" if strict_pass else "no",
        **deltas,
    }


def param_grid() -> Iterable[Params]:
    for cy_threshold in [410.0, 415.0, 418.0, 420.0]:
        for cell_111_right_gain in [-1.0, -0.5, 0.0, 0.5, 1.0]:
            for cell_111_down_gain in [3.0, 4.0, 5.0, 6.0]:
                for cell_211_right_gain in [0.0, 0.5, 1.0, 1.5, 2.0]:
                    for cell_211_down_gain in [-1.0, -0.5, 0.0, 0.25, 0.5, 0.75, 1.0]:
                        yield Params(
                            cy_threshold,
                            cell_111_right_gain,
                            cell_111_down_gain,
                            cell_211_right_gain,
                            cell_211_down_gain,
                        )


def aggregate(rows: list[dict[str, object]]) -> dict[str, object]:
    axis_keys = [
        "right_delta_rmse_m",
        "down_delta_rmse_m",
        "forward_delta_rmse_m",
        "position_delta_rmse_m",
    ]
    return {
        "trial_count": len(rows),
        "pass_trial_count": sum(
            1 for row in rows if row["pass_strict_rdf_all_axis"] == "yes"
        ),
        "sample_count": sum(int(row["sample_count"]) for row in rows),
        **{f"min_{key}": min(float(row[key]) for row in rows) for key in axis_keys},
        **{f"mean_{key}": float(np.mean([float(row[key]) for row in rows])) for key in axis_keys},
    }


def best_params(trials: list[Trial]) -> tuple[Params, dict[str, object], list[dict[str, object]]]:
    best: tuple[tuple[object, ...], Params, dict[str, object], list[dict[str, object]]] | None = None
    for params in param_grid():
        rows = [evaluate_trial(trial, params) for trial in trials]
        summary = aggregate(rows)
        min_margin = min(
            float(summary[f"min_{key}"])
            for key in [
                "right_delta_rmse_m",
                "down_delta_rmse_m",
                "forward_delta_rmse_m",
                "position_delta_rmse_m",
            ]
        )
        key = (
            int(summary["pass_trial_count"]),
            min_margin,
            float(summary["mean_position_delta_rmse_m"]),
        )
        if best is None or key > best[0]:
            best = (key, params, summary, rows)
    assert best is not None
    return best[1], best[2], best[3]


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    trials = load_trials(args.repo_root)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    fit_params, fit_summary, fit_rows = best_params(trials)
    write_csv(args.output_dir / "validation_fit_trial_summary.csv", fit_rows)

    holdout_rows: list[dict[str, object]] = []
    for held_out in trials:
        train = [
            trial
            for trial in trials
            if trial.report != held_out.report and trial.trial != held_out.trial
        ]
        params, train_summary, _ = best_params(train)
        row = evaluate_trial(held_out, params)
        row.update(
            {
                "selected_cy_threshold": params.cy_threshold,
                "selected_cell_111_right_gain": params.cell_111_right_gain,
                "selected_cell_111_down_gain": params.cell_111_down_gain,
                "selected_cell_211_right_gain": params.cell_211_right_gain,
                "selected_cell_211_down_gain": params.cell_211_down_gain,
                "train_pass_trial_count": train_summary["pass_trial_count"],
                "train_trial_count": train_summary["trial_count"],
            }
        )
        holdout_rows.append(row)

    holdout_summary = aggregate(holdout_rows)
    write_csv(args.output_dir / "report_trial_holdout_summary.csv", holdout_rows)

    metadata = {
        "method": "origin_bbox_cell_guard_holdout_validation",
        "coordinate_contract": {
            "frame": "stereo-center camera RDF",
            "axes": ["right_m", "down_m", "forward_m"],
            "position_metric": "Euclidean RMSE over RDF components",
            "frame_conversion": "none",
        },
        "fit_params": fit_params.__dict__,
        "validation_fit": fit_summary,
        "report_trial_holdout": holdout_summary,
        "reports": REPORTS,
    }
    (args.output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

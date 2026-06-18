#!/usr/bin/env python3
"""Search repaired-ROI composite candidates across two ROI validation reports.

Coordinate contract:
- Input comparison rows use stereo-center camera RDF axes in meters.
- ROI diagnostic rows contain corrected RDF error fields for the selected
  seed-specific Down observation.
- Right candidates and Forward candidates are combined in the same RDF frame.
- Output deltas are RMSE(raw_error_axis) - RMSE(composite_error_axis).
- Position is Euclidean RMSE over Right/Down/Forward in the same RDF frame.
- No NED/ENU/PX4/body conversion is performed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


def _find_stereo_distance_app() -> Path:
    if os.environ.get("STEREO_DISTANCE_APP"):
        return Path(os.environ["STEREO_DISTANCE_APP"]).expanduser()
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "StereoDistance"
        if (candidate / "wiris_stereo_distance").is_dir():
            return candidate
    return Path(__file__).resolve().parents[2] / "StereoDistance"


APP_DIR = _find_stereo_distance_app()
sys.path.insert(0, str(APP_DIR))

from wiris_stereo_distance.tracking_filter import (  # noqa: E402
    ConstantVelocityKalmanFilter,
    TrackingFilterConfig,
    measurement_variance_from_stereo,
)


DEFAULT_FX_PX = (1280.0 * 0.5) / math.tan(0.154997710698 * 0.5)


@dataclass(frozen=True)
class ReportSpec:
    label: str
    report_dir: Path
    roi_diagnostics: Path


@dataclass
class TrialData:
    label: str
    trial: int
    comparison_rows: list[dict[str, str]]
    roi_diagnostic_rows: list[dict[str, str]]
    raw_right_error: np.ndarray
    raw_down_error: np.ndarray
    raw_forward_error: np.ndarray
    roi_down_error: np.ndarray
    raw_range_m: np.ndarray
    origin_forward_error: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Combine repaired seed-specific ROI Down diagnostics with searched "
            "Right candidates and guarded origin-bbox Forward."
        )
    )
    parser.add_argument("--report", action="append", nargs=3, metavar=("LABEL", "REPORT_DIR", "ROI_DIAGNOSTICS"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--forward-gains", default="0.25,0.5,0.75,1,1.25,1.5,2")
    parser.add_argument("--forward-min-ranges-m", default="500")
    parser.add_argument(
        "--forward-range-thresholds-m",
        default="",
        help="Optional raw-range thresholds for two-gain Forward policy. If set, search low/high gains.",
    )
    parser.add_argument("--forward-low-gains", default="-2,-1,-0.5,-0.25,-0.1")
    parser.add_argument("--forward-high-gains", default="0.25,0.5,0.75,1,1.25,1.5,2")
    parser.add_argument("--tracker-accels", default="1,2,4,8")
    parser.add_argument("--tracker-min-lateral-stds-m", default="0.005,0.01")
    parser.add_argument("--tracker-min-forward-stds-m", default="0.5")
    parser.add_argument("--tracker-blends", default="0.01,0.02,0.03,0.04,0.05,0.075,0.1,0.15,0.2")
    parser.add_argument("--right-biases-m", default="-0.001,-0.0005,-0.0002,-0.0001,-0.00005,0,0.00005,0.0001,0.0002,0.0005,0.001")
    parser.add_argument("--image-width-px", type=float, default=1280.0)
    parser.add_argument("--baseline-m", type=float, default=25.0)
    parser.add_argument("--horizontal-fov-rad", type=float, default=0.154997710698)
    return parser.parse_args()


def parse_int_list(text: str) -> list[int]:
    return [int(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_float_list(text: str) -> list[float]:
    return [float(part.strip()) for part in str(text).split(",") if part.strip()]


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def rmse(values: Sequence[float] | np.ndarray) -> float:
    array = np.asarray(values, dtype=float)
    return math.sqrt(float(np.mean(array * array))) if array.size else float("nan")


def position_errors(right: np.ndarray, down: np.ndarray, forward: np.ndarray) -> np.ndarray:
    return np.sqrt(right * right + down * down + forward * forward)


def load_trials(spec: ReportSpec, trials: Sequence[int]) -> list[TrialData]:
    diagnostics_by_trial: dict[int, list[dict[str, str]]] = {}
    for row in read_csv(spec.roi_diagnostics):
        diagnostics_by_trial.setdefault(int(row["trial"]), []).append(row)

    result: list[TrialData] = []
    for trial in trials:
        comparison_rows = read_csv(spec.report_dir / f"validation_trial_{trial:02d}_comparison.csv")
        diagnostic_rows = diagnostics_by_trial[trial]
        if len(comparison_rows) != len(diagnostic_rows):
            raise ValueError(
                f"{spec.label} trial {trial} row mismatch: "
                f"comparison={len(comparison_rows)} roi={len(diagnostic_rows)}"
            )

        raw_range_m: list[float] = []
        origin_forward_error: list[float] = []
        for row in comparison_rows:
            raw_right = float(row["raw_right_m"])
            raw_down = float(row["raw_down_m"])
            raw_forward = float(row["raw_forward_m"])
            raw_range_m.append(math.sqrt(raw_right * raw_right + raw_down * raw_down + raw_forward * raw_forward))
            origin_forward = finite_float(row.get("origin_bbox_reprojection_corrected_forward_m"))
            origin_forward_error.append(
                float("nan") if origin_forward is None else origin_forward - float(row["truth_forward_m"])
            )

        result.append(
            TrialData(
                label=spec.label,
                trial=trial,
                comparison_rows=comparison_rows,
                roi_diagnostic_rows=diagnostic_rows,
                raw_right_error=np.array([float(row["raw_error_right_m"]) for row in comparison_rows]),
                raw_down_error=np.array([float(row["raw_error_down_m"]) for row in comparison_rows]),
                raw_forward_error=np.array([float(row["raw_error_forward_m"]) for row in comparison_rows]),
                roi_down_error=np.array([float(row["corrected_error_down_m"]) for row in diagnostic_rows]),
                raw_range_m=np.array(raw_range_m),
                origin_forward_error=np.array(origin_forward_error),
            )
        )
    return result


def tracker_delta_right(
    rows: Sequence[dict[str, str]],
    *,
    process_accel_std_mps2: float,
    min_lateral_std_m: float,
    min_forward_std_m: float,
    focal_length_px: float,
    baseline_m: float,
) -> np.ndarray:
    config = TrackingFilterConfig(
        process_accel_std_mps2=process_accel_std_mps2,
        initial_position_std_m=25.0,
        initial_velocity_std_mps=20.0,
        reset_after_s=3.0,
        max_normalized_innovation=1.0e9,
        min_lateral_std_m=min_lateral_std_m,
        min_forward_std_m=min_forward_std_m,
        max_lateral_std_m=50.0,
        max_forward_std_m=250.0,
    )
    tracker = ConstantVelocityKalmanFilter(config)
    deltas: list[float] = []
    for row in rows:
        point = (float(row["raw_right_m"]), float(row["raw_down_m"]), float(row["raw_forward_m"]))
        forward = abs(point[2])
        disparity = focal_length_px * baseline_m / forward if forward > 1.0e-6 else 0.0
        if disparity <= 0.0 or not math.isfinite(disparity):
            variance = (
                config.max_lateral_std_m**2,
                config.max_lateral_std_m**2,
                config.max_forward_std_m**2,
            )
        else:
            variance = measurement_variance_from_stereo(
                point_m=point,
                focal_length_px=focal_length_px,
                baseline_m=baseline_m,
                disparity_px=disparity,
                left_confidence=0.5,
                right_confidence=0.5,
                config=config,
            )
        result = tracker.update(
            timestamp_s=float(row["timestamp_s"]),
            point_m=point,
            measurement_variance_m2=variance,
        )
        deltas.append(float(result.point_m[0]) - point[0])
    return np.array(deltas)


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def evaluate(
    trial_data: Sequence[TrialData],
    *,
    right_error_by_trial: dict[tuple[str, int], np.ndarray],
    right_source: str,
    right_bias_m: float,
    forward_gain: float,
    forward_min_range_m: float,
    forward_low_gain: float | None = None,
    forward_high_gain: float | None = None,
    forward_range_threshold_m: float | None = None,
) -> tuple[dict[str, object], list[dict[str, object]], list[dict[str, object]]]:
    rows: list[dict[str, object]] = []
    sample_rows: list[dict[str, object]] = []
    for trial in trial_data:
        key = (trial.label, trial.trial)
        right_error = right_error_by_trial[key] - float(right_bias_m)
        down_error = trial.roi_down_error
        origin_available = np.isfinite(trial.origin_forward_error)
        if forward_range_threshold_m is None:
            use_forward = origin_available & (trial.raw_range_m >= float(forward_min_range_m))
            forward_error = np.where(
                use_forward,
                trial.raw_forward_error + float(forward_gain) * (trial.origin_forward_error - trial.raw_forward_error),
                trial.raw_forward_error,
            )
            forward_low_count = 0
            forward_high_count = int(use_forward.sum())
        else:
            low_mask = origin_available & (trial.raw_range_m < float(forward_range_threshold_m))
            high_mask = origin_available & (trial.raw_range_m >= float(forward_range_threshold_m))
            gains = np.where(low_mask, float(forward_low_gain), np.where(high_mask, float(forward_high_gain), 0.0))
            forward_error = trial.raw_forward_error + gains * (trial.origin_forward_error - trial.raw_forward_error)
            use_forward = low_mask | high_mask
            forward_low_count = int(low_mask.sum())
            forward_high_count = int(high_mask.sum())
        deltas = {
            "right_delta_rmse_m": rmse(trial.raw_right_error) - rmse(right_error),
            "down_delta_rmse_m": rmse(trial.raw_down_error) - rmse(down_error),
            "forward_delta_rmse_m": rmse(trial.raw_forward_error) - rmse(forward_error),
            "position_delta_rmse_m": rmse(
                position_errors(trial.raw_right_error, trial.raw_down_error, trial.raw_forward_error)
            )
            - rmse(position_errors(right_error, down_error, forward_error)),
        }
        rows.append(
            {
                "report": trial.label,
                "trial": trial.trial,
                "sample_count": int(trial.raw_right_error.size),
                "pass_strict_rdf_all_axis": "yes" if all(value > 0.0 for value in deltas.values()) else "no",
                **deltas,
                "right_source": right_source,
                "right_bias_m": f"{float(right_bias_m):.9g}",
                "forward_gain": f"{float(forward_gain):.9g}",
                "forward_min_range_m": f"{float(forward_min_range_m):.9g}",
                "forward_low_gain": "" if forward_low_gain is None else f"{float(forward_low_gain):.9g}",
                "forward_high_gain": "" if forward_high_gain is None else f"{float(forward_high_gain):.9g}",
                "forward_range_threshold_m": (
                    "" if forward_range_threshold_m is None else f"{float(forward_range_threshold_m):.9g}"
                ),
                "forward_corrected_sample_count": int(use_forward.sum()),
                "forward_low_sample_count": forward_low_count,
                "forward_high_sample_count": forward_high_count,
            }
        )
        for index in range(trial.raw_right_error.size):
            corrected_position = math.sqrt(
                float(right_error[index]) * float(right_error[index])
                + float(down_error[index]) * float(down_error[index])
                + float(forward_error[index]) * float(forward_error[index])
            )
            raw_position = math.sqrt(
                float(trial.raw_right_error[index]) * float(trial.raw_right_error[index])
                + float(trial.raw_down_error[index]) * float(trial.raw_down_error[index])
                + float(trial.raw_forward_error[index]) * float(trial.raw_forward_error[index])
            )
            roi_row = trial.roi_diagnostic_rows[index]
            sample_rows.append(
                {
                    "report": trial.label,
                    "trial": trial.trial,
                    "sample_index": index,
                    "fallback": roi_row.get("fallback", ""),
                    "range_bin": roi_row.get("range_bin", ""),
                    "screen_u_bin": roi_row.get("screen_u_bin", ""),
                    "screen_v_bin": roi_row.get("screen_v_bin", ""),
                    "raw_range_m": f"{float(trial.raw_range_m[index]):.9f}",
                    "raw_error_right_m": f"{float(trial.raw_right_error[index]):.9f}",
                    "raw_error_down_m": f"{float(trial.raw_down_error[index]):.9f}",
                    "raw_error_forward_m": f"{float(trial.raw_forward_error[index]):.9f}",
                    "raw_error_position_m": f"{raw_position:.9f}",
                    "corrected_error_right_m": f"{float(right_error[index]):.9f}",
                    "corrected_error_down_m": f"{float(down_error[index]):.9f}",
                    "corrected_error_forward_m": f"{float(forward_error[index]):.9f}",
                    "corrected_error_position_m": f"{corrected_position:.9f}",
                    "right_source": right_source,
                    "right_bias_m": f"{float(right_bias_m):.9g}",
                    "forward_gain": f"{float(forward_gain):.9g}",
                    "forward_min_range_m": f"{float(forward_min_range_m):.9g}",
                    "forward_low_gain": "" if forward_low_gain is None else f"{float(forward_low_gain):.9g}",
                    "forward_high_gain": "" if forward_high_gain is None else f"{float(forward_high_gain):.9g}",
                    "forward_range_threshold_m": (
                        "" if forward_range_threshold_m is None else f"{float(forward_range_threshold_m):.9g}"
                    ),
                }
            )
    summary = {
        "right_source": right_source,
        "right_bias_m": f"{float(right_bias_m):.9g}",
        "forward_gain": f"{float(forward_gain):.9g}",
        "forward_min_range_m": f"{float(forward_min_range_m):.9g}",
        "forward_low_gain": "" if forward_low_gain is None else f"{float(forward_low_gain):.9g}",
        "forward_high_gain": "" if forward_high_gain is None else f"{float(forward_high_gain):.9g}",
        "forward_range_threshold_m": "" if forward_range_threshold_m is None else f"{float(forward_range_threshold_m):.9g}",
        "report_count": len({row["report"] for row in rows}),
        "trial_count": len(rows),
        "sample_count": sum(int(row["sample_count"]) for row in rows),
        "pass_trial_count": sum(row["pass_strict_rdf_all_axis"] == "yes" for row in rows),
        "forward_corrected_sample_count": sum(int(row["forward_corrected_sample_count"]) for row in rows),
        "forward_low_sample_count": sum(int(row["forward_low_sample_count"]) for row in rows),
        "forward_high_sample_count": sum(int(row["forward_high_sample_count"]) for row in rows),
    }
    for axis in ("right", "down", "forward", "position"):
        values = [float(row[f"{axis}_delta_rmse_m"]) for row in rows]
        summary[f"min_{axis}_delta_rmse_m"] = min(values)
        summary[f"mean_{axis}_delta_rmse_m"] = float(np.mean(values))
    summary["pass_all_trials"] = "yes" if int(summary["pass_trial_count"]) == len(rows) else "no"
    return summary, rows, sample_rows


def main() -> int:
    args = parse_args()
    if not args.report:
        raise SystemExit("--report LABEL REPORT_DIR ROI_DIAGNOSTICS is required at least once")
    specs = [ReportSpec(label, Path(report_dir), Path(roi_diagnostics)) for label, report_dir, roi_diagnostics in args.report]
    trials = parse_int_list(args.validation_trials)
    trial_data = [trial for spec in specs for trial in load_trials(spec, trials)]
    focal_length_px = (float(args.image_width_px) * 0.5) / math.tan(float(args.horizontal_fov_rad) * 0.5)

    right_sources: list[tuple[str, dict[tuple[str, int], np.ndarray]]] = []
    raw_source = {(trial.label, trial.trial): trial.raw_right_error for trial in trial_data}
    right_sources.append(("raw", raw_source))

    for accel in parse_float_list(args.tracker_accels):
        for min_lateral in parse_float_list(args.tracker_min_lateral_stds_m):
            for min_forward in parse_float_list(args.tracker_min_forward_stds_m):
                base_delta = {
                    (trial.label, trial.trial): tracker_delta_right(
                        trial.comparison_rows,
                        process_accel_std_mps2=accel,
                        min_lateral_std_m=min_lateral,
                        min_forward_std_m=min_forward,
                        focal_length_px=focal_length_px,
                        baseline_m=float(args.baseline_m),
                    )
                    for trial in trial_data
                }
                for blend in parse_float_list(args.tracker_blends):
                    right_sources.append(
                        (
                            f"tracker_raw_right:accel={accel:g}:lat={min_lateral:g}:fstd={min_forward:g}:blend={blend:g}",
                            {
                                (trial.label, trial.trial): trial.raw_right_error
                                + float(blend) * base_delta[(trial.label, trial.trial)]
                                for trial in trial_data
                            },
                        )
                    )

    candidate_rows: list[dict[str, object]] = []
    best_rows: list[dict[str, object]] = []
    best_summary: dict[str, object] | None = None
    forward_specs: list[dict[str, float | None]] = []
    for forward_gain in parse_float_list(args.forward_gains):
        for forward_min_range in parse_float_list(args.forward_min_ranges_m):
            forward_specs.append(
                {
                    "forward_gain": forward_gain,
                    "forward_min_range_m": forward_min_range,
                    "forward_low_gain": None,
                    "forward_high_gain": None,
                    "forward_range_threshold_m": None,
                }
            )
    for threshold in parse_float_list(args.forward_range_thresholds_m):
        for low_gain in parse_float_list(args.forward_low_gains):
            for high_gain in parse_float_list(args.forward_high_gains):
                forward_specs.append(
                    {
                        "forward_gain": 0.0,
                        "forward_min_range_m": 0.0,
                        "forward_low_gain": low_gain,
                        "forward_high_gain": high_gain,
                        "forward_range_threshold_m": threshold,
                    }
                )

    for right_source, right_error_by_trial in right_sources:
        for right_bias in parse_float_list(args.right_biases_m):
            for forward_spec in forward_specs:
                    summary, rows, sample_rows = evaluate(
                        trial_data,
                        right_error_by_trial=right_error_by_trial,
                        right_source=right_source,
                        right_bias_m=right_bias,
                        forward_gain=float(forward_spec["forward_gain"] or 0.0),
                        forward_min_range_m=float(forward_spec["forward_min_range_m"] or 0.0),
                        forward_low_gain=forward_spec["forward_low_gain"],
                        forward_high_gain=forward_spec["forward_high_gain"],
                        forward_range_threshold_m=forward_spec["forward_range_threshold_m"],
                    )
                    candidate_rows.append(summary)
                    if best_summary is None or (
                        int(summary["pass_trial_count"]),
                        min(
                            float(summary[f"min_{axis}_delta_rmse_m"])
                            for axis in ("right", "down", "forward", "position")
                        ),
                        float(summary["mean_position_delta_rmse_m"]),
                    ) > (
                        int(best_summary["pass_trial_count"]),
                        min(
                            float(best_summary[f"min_{axis}_delta_rmse_m"])
                            for axis in ("right", "down", "forward", "position")
                        ),
                        float(best_summary["mean_position_delta_rmse_m"]),
                    ):
                        best_summary = summary
                        best_rows = rows
                        best_sample_rows = sample_rows

    candidate_rows.sort(
        key=lambda row: (
            int(row["pass_trial_count"]),
            min(float(row[f"min_{axis}_delta_rmse_m"]) for axis in ("right", "down", "forward", "position")),
            float(row["mean_position_delta_rmse_m"]),
        ),
        reverse=True,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "candidate_summary.csv", candidate_rows)
    write_csv(args.output_dir / "best_trial_summary.csv", best_rows)
    write_csv(args.output_dir / "best_sample_diagnostics.csv", best_sample_rows)
    (args.output_dir / "metadata.json").write_text(
        json.dumps(
            {
                "method": "roi_repaired_composite_right_search",
                "reports": [
                    {
                        "label": spec.label,
                        "report_dir": str(spec.report_dir),
                        "roi_diagnostics": str(spec.roi_diagnostics),
                    }
                    for spec in specs
                ],
                "validation_trials": trials,
                "coordinate_contract": "stereo-center camera RDF; no frame conversion",
                "best_candidate": best_summary,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    if best_summary:
        print(json.dumps(best_summary, indent=2, sort_keys=True))
        return 0 if best_summary.get("pass_all_trials") == "yes" else 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

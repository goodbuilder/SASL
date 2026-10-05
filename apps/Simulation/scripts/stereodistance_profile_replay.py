#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.runtime_guard_policy import (
    EstimatedToTruthAffineProfile,
    EstimatedToTruthDisparityProfile,
    EstimatedToTruthSim3Profile,
)


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
TRUTH_RDF_TRANSFORM_CONTRACT = (
    "truth_rdf must already be stereo_center_RDF from a declared T_source_to_stereo_rdf; "
    "direct NED-to-RDF axis alias is forbidden"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a StereoDistance runtime correction profile on recorded CSV rows. "
            "Uses named NED/RDF/FRD fields only; ambiguous x/y/z fields are ignored."
        )
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", default="", help="Optional split filter, e.g. validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    return parser.parse_args()


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def first_float(row: Mapping[str, str], names: Iterable[str]) -> float | None:
    for name in names:
        value = finite_float(row.get(name))
        if value is not None:
            return value
    return None


def first_vector(row: Mapping[str, str], alternatives: Sequence[tuple[str, str, str]]) -> np.ndarray | None:
    for names in alternatives:
        values = [finite_float(row.get(name)) for name in names]
        if all(value is not None for value in values):
            return np.array(values, dtype=float)
    return None


def truth_rdf_from_row(row: Mapping[str, str]) -> np.ndarray | None:
    truth_rdf = first_vector(
        row,
        (
            ("truth_right_m", "truth_down_m", "truth_forward_m"),
            ("gt_right_m", "gt_down_m", "gt_forward_m"),
            ("drone_right_m", "drone_down_m", "drone_forward_m"),
        ),
    )
    if truth_rdf is not None:
        return truth_rdf
    return None


def raw_rdf_from_row(row: Mapping[str, str]) -> np.ndarray | None:
    # Recorder/public CSV fields are named forward/right/down, but the runtime
    # profile consumes internal RDF: right/down/forward.
    raw_rdf = first_vector(
        row,
        (
            ("raw_right_m", "raw_down_m", "raw_forward_m"),
            ("estimated_right_m", "estimated_down_m", "estimated_forward_m"),
        ),
    )
    if raw_rdf is not None:
        return raw_rdf
    forward = first_float(row, ("raw_forward_m", "estimated_forward_m"))
    right = first_float(row, ("raw_right_m", "estimated_right_m"))
    down = first_float(row, ("raw_down_m", "estimated_down_m"))
    if None in (forward, right, down):
        return None
    return np.array([right, down, forward], dtype=float)


def disparity_from_row(row: Mapping[str, str]) -> float | None:
    return first_float(
        row,
        (
            "measured_disparity_px",
            "raw_disparity_px",
            "disparity_px",
            "comparison_disparity_px",
        ),
    )


def range_m(point: np.ndarray) -> float:
    return float(np.linalg.norm(point))


def rmse(values: Sequence[float]) -> float:
    arr = np.asarray(values, dtype=float)
    return float(math.sqrt(np.mean(arr * arr))) if arr.size else float("nan")


def percentile(values: Sequence[float], q: float) -> float:
    arr = np.asarray(values, dtype=float)
    return float(np.percentile(arr, q)) if arr.size else float("nan")


def load_profile(path: Path) -> EstimatedToTruthSim3Profile | EstimatedToTruthDisparityProfile | EstimatedToTruthAffineProfile:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("mode") == EstimatedToTruthSim3Profile.mode:
        return EstimatedToTruthSim3Profile.from_json_dict(payload)
    if payload.get("mode") == EstimatedToTruthDisparityProfile.mode:
        return EstimatedToTruthDisparityProfile.from_json_dict(payload)
    if payload.get("mode") == EstimatedToTruthAffineProfile.mode:
        return EstimatedToTruthAffineProfile.from_json_dict(payload)
    raise ValueError(f"unsupported runtime profile mode: {payload.get('mode')!r}")


def replay_rows(
    rows: Sequence[Mapping[str, str]],
    profile: EstimatedToTruthSim3Profile | EstimatedToTruthDisparityProfile | EstimatedToTruthAffineProfile,
    *,
    split_filter: str,
    target_range_m: float,
    target_window_m: float,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    predictions: list[dict[str, object]] = []
    raw_range_errors: list[float] = []
    corrected_range_errors: list[float] = []
    raw_axis_errors: list[np.ndarray] = []
    corrected_axis_errors: list[np.ndarray] = []
    target_corrected_errors: list[float] = []
    target_raw_errors: list[float] = []
    skipped = 0

    for row_index, row in enumerate(rows):
        split = str(row.get("split", ""))
        if split_filter and split != split_filter:
            continue
        truth = truth_rdf_from_row(row)
        raw = raw_rdf_from_row(row)
        disparity = disparity_from_row(row)
        if truth is None or raw is None or disparity is None:
            skipped += 1
            continue
        raw_payload = {
            "right_m": float(raw[0]),
            "down_m": float(raw[1]),
            "forward_m": float(raw[2]),
            "range_m": range_m(raw),
            "disparity_px": float(disparity),
        }
        left_u = row.get("left_u_px") or row.get("left_cx_px") or row.get("left_cx")
        left_v = row.get("left_v_px") or row.get("left_cy_px") or row.get("left_cy")
        right_u = row.get("right_u_px") or row.get("right_cx_px") or row.get("right_cx")
        right_v = row.get("right_v_px") or row.get("right_cy_px") or row.get("right_cy")
        if left_u not in (None, "") and left_v not in (None, ""):
            raw_payload["left_u_px"] = float(left_u)
            raw_payload["left_v_px"] = float(left_v)
        if right_u not in (None, "") and right_v not in (None, ""):
            raw_payload["right_u_px"] = float(right_u)
            raw_payload["right_v_px"] = float(right_v)
        result = profile.apply(raw_payload)
        if result.corrected is None:
            skipped += 1
            continue
        corrected = np.array(
            [
                result.corrected["right_m"],
                result.corrected["down_m"],
                result.corrected["forward_m"],
            ],
            dtype=float,
        )
        truth_range = range_m(truth)
        raw_range = range_m(raw)
        corrected_range = range_m(corrected)
        raw_error = raw - truth
        corrected_error = corrected - truth
        raw_range_error = raw_range - truth_range
        corrected_range_error = corrected_range - truth_range
        raw_range_errors.append(raw_range_error)
        corrected_range_errors.append(corrected_range_error)
        raw_axis_errors.append(raw_error)
        corrected_axis_errors.append(corrected_error)
        if abs(truth_range - target_range_m) <= target_window_m:
            target_raw_errors.append(raw_range_error)
            target_corrected_errors.append(corrected_range_error)
        predictions.append(
            {
                "row_index": row_index,
                "sample_id": row.get("sample_id", row_index),
                "split": split,
                "rdf_axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "truth_right_m": f"{truth[0]:.9f}",
                "truth_down_m": f"{truth[1]:.9f}",
                "truth_forward_m": f"{truth[2]:.9f}",
                "truth_range_m": f"{truth_range:.9f}",
                "raw_right_m": f"{raw[0]:.9f}",
                "raw_down_m": f"{raw[1]:.9f}",
                "raw_forward_m": f"{raw[2]:.9f}",
                "raw_range_m": f"{raw_range:.9f}",
                "corrected_right_m": f"{corrected[0]:.9f}",
                "corrected_down_m": f"{corrected[1]:.9f}",
                "corrected_forward_m": f"{corrected[2]:.9f}",
                "corrected_range_m": f"{corrected_range:.9f}",
                "raw_range_error_m": f"{raw_range_error:.9f}",
                "corrected_range_error_m": f"{corrected_range_error:.9f}",
                "raw_right_error_m": f"{raw_error[0]:.9f}",
                "raw_down_error_m": f"{raw_error[1]:.9f}",
                "raw_forward_error_m": f"{raw_error[2]:.9f}",
                "corrected_right_error_m": f"{corrected_error[0]:.9f}",
                "corrected_down_error_m": f"{corrected_error[1]:.9f}",
                "corrected_forward_error_m": f"{corrected_error[2]:.9f}",
                "measured_disparity_px": f"{float(disparity):.9f}",
            }
        )

    raw_axis = np.vstack(raw_axis_errors) if raw_axis_errors else np.empty((0, 3))
    corrected_axis = np.vstack(corrected_axis_errors) if corrected_axis_errors else np.empty((0, 3))
    summary = {
        "profile_id": profile.profile_id,
        "profile_mode": profile.mode,
        "input_row_count": len(rows),
        "evaluated_row_count": len(predictions),
        "skipped_row_count": skipped,
        "split_filter": split_filter,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "truth_rdf_transform_contract": TRUTH_RDF_TRANSFORM_CONTRACT,
        "direct_ned_to_rdf_axis_alias_allowed": False,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "validation_range_rmse_raw_m": rmse(raw_range_errors),
        "validation_range_rmse_corrected_m": rmse(corrected_range_errors),
        "validation_range_median_abs_raw_m": percentile([abs(value) for value in raw_range_errors], 50),
        "validation_range_median_abs_corrected_m": percentile([abs(value) for value in corrected_range_errors], 50),
        "validation_range_p90_abs_raw_m": percentile([abs(value) for value in raw_range_errors], 90),
        "validation_range_p90_abs_corrected_m": percentile([abs(value) for value in corrected_range_errors], 90),
        "validation_position_rmse_raw_m": rmse([np.linalg.norm(value) for value in raw_axis]),
        "validation_position_rmse_corrected_m": rmse([np.linalg.norm(value) for value in corrected_axis]),
        "validation_position_median_raw_m": percentile([np.linalg.norm(value) for value in raw_axis], 50),
        "validation_position_median_corrected_m": percentile([np.linalg.norm(value) for value in corrected_axis], 50),
        "validation_position_p90_raw_m": percentile([np.linalg.norm(value) for value in raw_axis], 90),
        "validation_position_p90_corrected_m": percentile([np.linalg.norm(value) for value in corrected_axis], 90),
        "validation_right_rmse_raw_m": rmse(raw_axis[:, 0]) if raw_axis.size else float("nan"),
        "validation_right_rmse_corrected_m": rmse(corrected_axis[:, 0]) if corrected_axis.size else float("nan"),
        "validation_down_rmse_raw_m": rmse(raw_axis[:, 1]) if raw_axis.size else float("nan"),
        "validation_down_rmse_corrected_m": rmse(corrected_axis[:, 1]) if corrected_axis.size else float("nan"),
        "validation_forward_rmse_raw_m": rmse(raw_axis[:, 2]) if raw_axis.size else float("nan"),
        "validation_forward_rmse_corrected_m": rmse(corrected_axis[:, 2]) if corrected_axis.size else float("nan"),
        "target_range_m": target_range_m,
        "target_window_half_width_m": target_window_m,
        "target_window_count": len(target_corrected_errors),
        "target_abs_mean_error_raw_m": float(abs(np.mean(target_raw_errors))) if target_raw_errors else float("inf"),
        "target_abs_mean_error_corrected_m": (
            float(abs(np.mean(target_corrected_errors))) if target_corrected_errors else float("inf")
        ),
        "target_max_abs_error_raw_m": (
            float(np.max(np.abs(target_raw_errors))) if target_raw_errors else float("inf")
        ),
        "target_max_abs_error_corrected_m": (
            float(np.max(np.abs(target_corrected_errors))) if target_corrected_errors else float("inf")
        ),
        "runtime_truth_used": False,
    }
    return predictions, summary


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, summary: Mapping[str, object]) -> None:
    lines = ["StereoDistance runtime profile replay", ""]
    for key, value in summary.items():
        if isinstance(value, float):
            value_text = "inf" if math.isinf(value) else f"{value:.9f}"
        else:
            value_text = str(value)
        lines.append(f"{key}: {value_text}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_range_errors(path: Path, predictions: Sequence[Mapping[str, object]]) -> None:
    if not predictions:
        return
    truth_range = np.array([float(row["truth_range_m"]) for row in predictions], dtype=float)
    raw_error = np.array([float(row["raw_range_error_m"]) for row in predictions], dtype=float)
    corrected_error = np.array([float(row["corrected_range_error_m"]) for row in predictions], dtype=float)
    order = np.argsort(truth_range)
    plt.figure(figsize=(11, 6))
    plt.plot(truth_range[order], raw_error[order], label="raw")
    plt.plot(truth_range[order], corrected_error[order], label="corrected")
    plt.axhline(10.0, color="tab:red", linestyle="--", linewidth=1)
    plt.axhline(-10.0, color="tab:red", linestyle="--", linewidth=1)
    plt.xlabel("truth range (m)")
    plt.ylabel("range error (m)")
    plt.title("Runtime profile replay range error")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = load_profile(args.profile_json)
    with args.input_csv.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    predictions, summary = replay_rows(
        rows,
        profile,
        split_filter=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    if not predictions:
        raise SystemExit("no rows were replayed; check CSV named fields and split filter")
    write_csv(args.output_dir / "profile_replay_predictions.csv", predictions)
    write_csv(args.output_dir / "profile_replay_summary.csv", [summary])
    write_summary(args.output_dir / "profile_replay_summary.txt", summary)
    plot_range_errors(args.output_dir / "profile_replay_range_error.png", predictions)
    print(f"wrote replay results to {args.output_dir}")
    print(
        "range RMSE raw -> corrected: "
        f"{summary['validation_range_rmse_raw_m']:.3f} -> "
        f"{summary['validation_range_rmse_corrected_m']:.3f} m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from export_stereodistance_profile_by_operating_mode import has_truth_fields  # noqa: E402
from select_stereodistance_profile_by_calibration_replay import read_csv, write_json  # noqa: E402
from select_stereodistance_profile_by_runtime_measurement_guard import (  # noqa: E402
    DEFAULT_INTRINSIC_SCALE_GAP_PRODUCT_RATIO_P10_THRESHOLD,
    runtime_measurement_features,
)
from select_stereodistance_profile_from_catalog import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit and export a StereoDistance runtime profile from a calibration CSV when the "
            "runtime-safe disparity-forward product-ratio guard indicates an intrinsic-scale regime. "
            "Truth is used only for this calibration-time fitting step; the exported profile consumes "
            "raw stereo-center RDF/disparity at runtime."
        )
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--seed", type=int, default=20260613)
    parser.add_argument(
        "--product-ratio-p10-threshold",
        type=float,
        default=DEFAULT_INTRINSIC_SCALE_GAP_PRODUCT_RATIO_P10_THRESHOLD,
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Fit/export even when the product-ratio guard does not detect an intrinsic-scale gap.",
    )
    parser.add_argument(
        "--pipeline-script",
        type=Path,
        default=SCRIPT_DIR / "rtk_stereo_calibration_pipeline.py",
    )
    return parser.parse_args()


def selected_runtime_profile_path(fit_dir: Path) -> Path:
    recommended = fit_dir / "sim3_runtime_profile.json"
    if recommended.is_file():
        return recommended
    manifest_path = fit_dir / "runtime_profiles" / "manifest.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        for row in manifest if isinstance(manifest, list) else []:
            if isinstance(row, Mapping) and bool(row.get("selected")):
                relative = str(row.get("profile_path", ""))
                if relative:
                    candidate = fit_dir / relative
                    if candidate.is_file():
                        return candidate
    raise FileNotFoundError(f"no selected runtime profile found under {fit_dir}")


def run_pipeline(
    *,
    pipeline_script: Path,
    input_csv: Path,
    fit_dir: Path,
    target_range_m: float,
    target_window_m: float,
    train_ratio: float,
    seed: int,
) -> subprocess.CompletedProcess[str]:
    command = [
        sys.executable,
        str(pipeline_script),
        "--mode",
        "csv",
        "--input-csv",
        str(input_csv),
        "--output-dir",
        str(fit_dir),
        "--target-range-m",
        str(target_range_m),
        "--target-window-m",
        str(target_window_m),
        "--train-ratio",
        str(train_ratio),
        "--seed",
        str(seed),
    ]
    return subprocess.run(command, check=True, text=True, capture_output=True)


def write_summary_txt(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Intrinsic-scale calibration profile export",
        "",
        f"selected: {summary.get('selected', '')}",
        f"input_csv: {summary.get('input_csv', '')}",
        f"output_profile: {summary.get('output_profile', '')}",
        f"product_ratio_p10_to_nominal: {summary.get('product_ratio_p10_to_nominal', '')}",
        f"product_ratio_p10_threshold: {summary.get('product_ratio_p10_threshold', '')}",
        f"intrinsic_scale_gap_detected: {summary.get('intrinsic_scale_gap_detected', '')}",
        f"recommended_method: {summary.get('recommended_method', '')}",
        f"recommendation_reason: {summary.get('recommendation_reason', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_intrinsic_scale_profile(args: argparse.Namespace) -> dict[str, object]:
    if not args.input_csv.is_file():
        raise ValueError(f"input CSV does not exist: {args.input_csv}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = read_csv(args.input_csv)
    features = runtime_measurement_features(rows)
    ratio_p10 = float(features.get("disparity_forward_product_ratio_p10_to_nominal", float("nan")))
    detected = ratio_p10 >= float(args.product_ratio_p10_threshold)
    if not has_truth_fields(args.input_csv):
        raise ValueError("calibration-time intrinsic profile export requires truth fields")
    summary: dict[str, object] = {
        "mode": "intrinsic_scale_calibration_profile_export",
        "input_csv": str(args.input_csv),
        "output_dir": str(args.output_dir),
        "output_profile": str(args.output_profile),
        "target_range_m": float(args.target_range_m),
        "target_window_m": float(args.target_window_m),
        "target_error_limit_m": float(args.target_error_limit_m),
        "product_ratio_p10_to_nominal": ratio_p10,
        "product_ratio_p10_threshold": float(args.product_ratio_p10_threshold),
        "intrinsic_scale_gap_detected": bool(detected),
        "force": bool(args.force),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Runtime-safe product-ratio features decide whether this calibration export is needed; "
            "PX4/RTK truth is used only inside the calibration-time fitting pipeline."
        ),
        "runtime_input_contract": "exported profile consumes raw stereo-center RDF/disparity only",
    }
    if not detected and not args.force:
        summary["selected"] = False
        summary["skip_reason"] = "intrinsic_scale_gap_not_detected"
        write_json(args.output_dir / "intrinsic_scale_profile_export_summary.json", summary)
        write_summary_txt(args.output_dir / "intrinsic_scale_profile_export_summary.txt", summary)
        return summary

    fit_dir = args.output_dir / "calibration_fit"
    completed = run_pipeline(
        pipeline_script=args.pipeline_script,
        input_csv=args.input_csv,
        fit_dir=fit_dir,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        train_ratio=args.train_ratio,
        seed=args.seed,
    )
    source_profile = selected_runtime_profile_path(fit_dir)
    args.output_profile.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source_profile, args.output_profile)
    summary.update(
        {
            "selected": True,
            "fit_dir": str(fit_dir),
            "source_profile": str(source_profile),
            "pipeline_stdout": completed.stdout,
            "pipeline_stderr": completed.stderr,
        }
    )
    run_config = fit_dir / "run_config.json"
    if run_config.is_file():
        payload = json.loads(run_config.read_text(encoding="utf-8"))
        summary["recommended_method"] = payload.get("recommendation", "")
        summary["recommendation_reason"] = payload.get("recommendation_reason", "")
    write_json(args.output_dir / "intrinsic_scale_profile_export_summary.json", summary)
    write_summary_txt(args.output_dir / "intrinsic_scale_profile_export_summary.txt", summary)
    return summary


def main() -> int:
    summary = export_intrinsic_scale_profile(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

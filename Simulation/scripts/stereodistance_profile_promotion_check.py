#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.runtime_guard_policy import (
    EstimatedToTruthAffineProfile,
    EstimatedToTruthDisparityProfile,
    EstimatedToTruthSim3Profile,
)
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
FORBIDDEN_RUNTIME_INPUTS = {
    "calibration_truth",
    "px4_rtk_truth",
    "true_disparity",
    "true_image_feature_location",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check that an exported StereoDistance runtime profile is promotion-ready: "
            "schema loads, the real StereoDistance profile loader accepts it, and sample "
            "runtime rows can be corrected without truth inputs."
        )
    )
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--sample-csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--max-samples", type=int, default=50)
    return parser.parse_args()


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def first_float(row: Mapping[str, str], names: Sequence[str]) -> float | None:
    for name in names:
        value = finite_float(row.get(name))
        if value is not None:
            return value
    return None


def raw_rdf_from_row(row: Mapping[str, str]) -> tuple[float, float, float] | None:
    right = first_float(row, ("raw_right_m", "estimated_right_m"))
    down = first_float(row, ("raw_down_m", "estimated_down_m"))
    forward = first_float(row, ("raw_forward_m", "estimated_forward_m"))
    if None in (right, down, forward):
        return None
    return float(right), float(down), float(forward)


def disparity_from_row(row: Mapping[str, str]) -> float | None:
    return first_float(row, ("measured_disparity_px", "raw_disparity_px", "disparity_px"))


def left_image_point_from_row(row: Mapping[str, str]) -> tuple[float, float] | None:
    left_u = first_float(row, ("left_u_px", "left_cx_px", "left_cx"))
    left_v = first_float(row, ("left_v_px", "left_cy_px", "left_cy"))
    if None in (left_u, left_v):
        return None
    return float(left_u), float(left_v)


def load_profile_payload(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


PROFILE_CLASSES = (EstimatedToTruthSim3Profile, EstimatedToTruthDisparityProfile, EstimatedToTruthAffineProfile)


def load_runtime_profile(payload: Mapping[str, object]):
    mode = payload.get("mode")
    if mode == EstimatedToTruthSim3Profile.mode:
        return EstimatedToTruthSim3Profile.from_json_dict(payload)
    if mode == EstimatedToTruthDisparityProfile.mode:
        return EstimatedToTruthDisparityProfile.from_json_dict(payload)
    if mode == EstimatedToTruthAffineProfile.mode:
        return EstimatedToTruthAffineProfile.from_json_dict(payload)
    raise ValueError(f"unsupported runtime profile mode: {mode!r}")


def node_loader_check(path: Path) -> tuple[bool, str]:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.enable_bbox_center_adjustment = True
    node.bbox_center_adjustment_path = str(path)
    messages: list[str] = []
    node.get_logger = lambda: type(
        "Logger",
        (),
        {
            "warning": lambda self, msg: messages.append(f"WARNING: {msg}"),
            "info": lambda self, msg: messages.append(f"INFO: {msg}"),
        },
    )()
    loaded = StereoDistanceNode._load_bbox_center_adjustment(node)
    return isinstance(loaded, PROFILE_CLASSES), "\n".join(messages)


def apply_sample_rows(
    profile: EstimatedToTruthSim3Profile | EstimatedToTruthDisparityProfile | EstimatedToTruthAffineProfile,
    sample_csv: Path | None,
    *,
    split: str,
    max_samples: int,
) -> tuple[int, int, list[dict[str, object]]]:
    if sample_csv is None:
        return 0, 0, []
    rows_out: list[dict[str, object]] = []
    invalid = 0
    with sample_csv.open(newline="", encoding="utf-8") as handle:
        for row_index, row in enumerate(csv.DictReader(handle)):
            if split and str(row.get("split", "")) != split:
                continue
            raw = raw_rdf_from_row(row)
            disparity = disparity_from_row(row)
            if raw is None or disparity is None:
                invalid += 1
                continue
            raw_payload = {
                "right_m": raw[0],
                "down_m": raw[1],
                "forward_m": raw[2],
                "range_m": math.sqrt(raw[0] ** 2 + raw[1] ** 2 + raw[2] ** 2),
                "disparity_px": disparity,
            }
            left_image_point = left_image_point_from_row(row)
            if left_image_point is not None:
                raw_payload["left_u_px"] = left_image_point[0]
                raw_payload["left_v_px"] = left_image_point[1]
            result = profile.apply(raw_payload)
            if result.corrected is None:
                invalid += 1
                continue
            corrected = result.corrected
            finite = all(
                math.isfinite(float(corrected[key]))
                for key in ("right_m", "down_m", "forward_m", "range_m", "disparity_px")
            )
            if not finite:
                invalid += 1
                continue
            rows_out.append(
                {
                    "row_index": row_index,
                    "split": row.get("split", ""),
                    "raw_right_m": raw[0],
                    "raw_down_m": raw[1],
                    "raw_forward_m": raw[2],
                    "corrected_right_m": corrected["right_m"],
                    "corrected_down_m": corrected["down_m"],
                    "corrected_forward_m": corrected["forward_m"],
                    "corrected_range_m": corrected["range_m"],
                    "profile_fallback_level": result.status.get("fallback_level"),
                    "profile_fallback_reason": result.status.get("fallback_reason"),
                    "left_u_px": raw_payload.get("left_u_px", ""),
                    "left_v_px": raw_payload.get("left_v_px", ""),
                    "rdf_axis_contract": RDF_AXIS_CONTRACT,
                    "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                }
            )
            if len(rows_out) >= max_samples:
                break
    return len(rows_out), invalid, rows_out


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_text_summary(path: Path, summary: Mapping[str, object]) -> None:
    lines = ["StereoDistance runtime profile promotion check", ""]
    for key, value in summary.items():
        if isinstance(value, (dict, list)):
            lines.append(f"{key}: {json.dumps(value, ensure_ascii=False)}")
        else:
            lines.append(f"{key}: {value}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_check(
    profile_json: Path,
    output_dir: Path,
    *,
    sample_csv: Path | None = None,
    split: str = "validation",
    max_samples: int = 50,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = load_profile_payload(profile_json)
    schema_load_ok = False
    schema_error = ""
    profile: EstimatedToTruthSim3Profile | EstimatedToTruthDisparityProfile | EstimatedToTruthAffineProfile | None = None
    try:
        profile = load_runtime_profile(payload)
        schema_load_ok = True
    except Exception as exc:
        schema_error = str(exc)

    node_load_ok, node_loader_log = node_loader_check(profile_json)
    forbidden_runtime_inputs = set(str(item) for item in payload.get("forbidden_runtime_inputs", []))
    forbidden_declared = FORBIDDEN_RUNTIME_INPUTS.issubset(forbidden_runtime_inputs)
    runtime_inputs = [str(item) for item in payload.get("runtime_inputs", [])]
    runtime_truth_used = any(item in FORBIDDEN_RUNTIME_INPUTS for item in runtime_inputs)

    applied_count = 0
    invalid_count = 0
    sample_rows: list[dict[str, object]] = []
    if profile is not None:
        applied_count, invalid_count, sample_rows = apply_sample_rows(
            profile,
            sample_csv,
            split=split,
            max_samples=max_samples,
        )
    sample_check_ok = sample_csv is None or applied_count > 0
    pass_check = bool(
        schema_load_ok
        and node_load_ok
        and forbidden_declared
        and not runtime_truth_used
        and sample_check_ok
    )
    summary: dict[str, object] = {
        "pass_promotion_check": pass_check,
        "profile_path": str(profile_json),
        "profile_id": payload.get("profile_id", ""),
        "profile_mode": payload.get("mode", ""),
        "schema_load_ok": schema_load_ok,
        "schema_error": schema_error,
        "node_loader_ok": node_load_ok,
        "node_loader_log": node_loader_log,
        "runtime_truth_used": runtime_truth_used,
        "forbidden_runtime_inputs_declared": forbidden_declared,
        "sample_csv": str(sample_csv) if sample_csv else "",
        "sample_split": split,
        "sample_apply_count": applied_count,
        "sample_invalid_count": invalid_count,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }
    (output_dir / "promotion_check.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_text_summary(output_dir / "promotion_check.txt", summary)
    write_csv(output_dir / "promotion_check_samples.csv", sample_rows)
    return summary


def main() -> int:
    args = parse_args()
    summary = run_check(
        args.profile_json,
        args.output_dir,
        sample_csv=args.sample_csv,
        split=args.split,
        max_samples=args.max_samples,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["pass_promotion_check"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

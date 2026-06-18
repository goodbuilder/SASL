#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
TRUTH_RDF_TRANSFORM_CONTRACT = (
    "explicit_transform_required: truth_rdf must be stereo_center_RDF from a declared "
    "T_source_to_stereo_rdf or an already stereo-frame truth producer; direct source-frame "
    "axis aliasing is forbidden"
)

TRUTH_RDF_FIELDS = (
    ("truth_right_m", "truth_down_m", "truth_forward_m"),
    ("drone_right_m", "drone_down_m", "drone_forward_m"),
    ("gt_right_m", "gt_down_m", "gt_forward_m"),
)
TRUTH_NED_FIELDS = (
    ("truth_north_m", "truth_east_m", "truth_down_ned_m"),
    ("north_m", "east_m", "down_m"),
)
RAW_RDF_FIELDS = (
    ("raw_right_m", "raw_down_m", "raw_forward_m"),
    ("estimated_right_m", "estimated_down_m", "estimated_forward_m"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Preflight a calibration CSV before running estimated-to-truth fitting. "
            "Uses the same named NED/RDF/FRD field contract as the calibration pipeline."
        )
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--min-target-count", type=int, default=3)
    return parser.parse_args()


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def first_vector(
    row: Mapping[str, str],
    alternatives: Sequence[tuple[str, str, str]],
) -> tuple[tuple[str, str, str] | None, list[float] | None]:
    for names in alternatives:
        values = [finite_float(row.get(name)) for name in names]
        if all(value is not None for value in values):
            return names, [float(value) for value in values if value is not None]
    return None, None


def first_float(row: Mapping[str, str], names: Iterable[str]) -> tuple[str | None, float | None]:
    for name in names:
        value = finite_float(row.get(name))
        if value is not None:
            return name, value
    return None, None


def range_m(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def row_diagnostic(
    row_index: int,
    row: Mapping[str, str],
    *,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    truth_source, truth_rdf = first_vector(row, TRUTH_RDF_FIELDS)
    truth_ned_source, truth_ned = first_vector(row, TRUTH_NED_FIELDS)

    raw_source, raw_rdf = first_vector(row, RAW_RDF_FIELDS)
    left_u_name, left_u = first_float(row, ("left_u_px", "left_cx_px", "left_cx"))
    left_v_name, left_v = first_float(row, ("left_v_px", "left_cy_px", "left_cy"))
    right_u_name, right_u = first_float(row, ("right_u_px", "right_cx_px", "right_cx"))
    disparity_name, disparity = first_float(row, ("measured_disparity_px", "raw_disparity_px", "disparity_px"))
    disparity_from_centers = False
    if disparity is None and left_u is not None and right_u is not None:
        disparity = abs(left_u - right_u)
        disparity_name = "abs(left_u-right_u)"
        disparity_from_centers = True

    reasons: list[str] = []
    if truth_rdf is None:
        reasons.append("missing_truth_rdf")
    if truth_rdf is None and truth_ned is not None:
        reasons.append("missing_explicit_T_source_to_stereo_rdf")
    if raw_rdf is None and None in (left_u, left_v, disparity):
        reasons.append("missing_raw_rdf_or_triangulation_inputs")
    if disparity is None:
        reasons.append("missing_measured_disparity")
    if disparity is not None and disparity <= 1.0e-9:
        reasons.append("nonpositive_disparity")

    truth_range = range_m(truth_rdf) if truth_rdf is not None else float("nan")
    target_window = bool(abs(truth_range - target_range_m) <= target_window_m) if math.isfinite(truth_range) else False
    usable = not reasons
    return {
        "row_index": row_index,
        "usable": usable,
        "reasons": ";".join(reasons),
        "truth_source": ",".join(truth_source) if truth_source else "",
        "truth_ned_source": ",".join(truth_ned_source) if truth_ned_source else "",
        "raw_source": ",".join(raw_source) if raw_source else "triangulation" if raw_rdf is None and usable else "",
        "left_u_field": left_u_name or "",
        "left_v_field": left_v_name or "",
        "right_u_field": right_u_name or "",
        "disparity_field": disparity_name or "",
        "disparity_from_centers": disparity_from_centers,
        "truth_range_m": truth_range,
        "target_window": target_window,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "ned_axis_contract": NED_AXIS_CONTRACT,
    }


def summarize(diagnostics: Sequence[Mapping[str, object]], *, min_target_count: int) -> dict[str, object]:
    total = len(diagnostics)
    usable = [row for row in diagnostics if bool(row["usable"])]
    target = [row for row in usable if bool(row["target_window"])]
    reason_counts: dict[str, int] = {}
    for row in diagnostics:
        reasons = str(row.get("reasons") or "")
        if not reasons:
            continue
        for reason in reasons.split(";"):
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
    truth_ranges = [float(row["truth_range_m"]) for row in usable if math.isfinite(float(row["truth_range_m"]))]
    pass_preflight = bool(usable) and len(target) >= min_target_count
    return {
        "pass_preflight": pass_preflight,
        "row_count": total,
        "usable_row_count": len(usable),
        "skipped_row_count": total - len(usable),
        "target_window_usable_count": len(target),
        "target_window_min_count": min_target_count,
        "truth_range_min_m": min(truth_ranges) if truth_ranges else float("nan"),
        "truth_range_max_m": max(truth_ranges) if truth_ranges else float("nan"),
        "reason_counts": reason_counts,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "ned_axis_contract": NED_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": TRUTH_RDF_TRANSFORM_CONTRACT,
        "direct_source_axis_alias_allowed": False,
        "direct_ned_to_rdf_axis_alias_allowed": False,
        "ned_to_rdf_contract": TRUTH_RDF_TRANSFORM_CONTRACT,
    }


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary_txt(path: Path, summary: Mapping[str, object]) -> None:
    lines = ["Calibration CSV preflight", ""]
    for key, value in summary.items():
        if isinstance(value, dict):
            lines.append(f"{key}:")
            for sub_key, sub_value in sorted(value.items()):
                lines.append(f"- {sub_key}: {sub_value}")
        else:
            lines.append(f"{key}: {value}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_preflight(
    input_csv: Path,
    output_dir: Path,
    *,
    target_range_m: float,
    target_window_m: float,
    min_target_count: int,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    with input_csv.open(newline="", encoding="utf-8") as handle:
        diagnostics = [
            row_diagnostic(
                index,
                row,
                target_range_m=target_range_m,
                target_window_m=target_window_m,
            )
            for index, row in enumerate(csv.DictReader(handle))
        ]
    summary = summarize(diagnostics, min_target_count=min_target_count)
    write_csv(output_dir / "preflight_rows.csv", diagnostics)
    (output_dir / "preflight_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_summary_txt(output_dir / "preflight_summary.txt", summary)
    return summary


def main() -> int:
    args = parse_args()
    summary = run_preflight(
        args.input_csv,
        args.output_dir,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        min_target_count=args.min_target_count,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["pass_preflight"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

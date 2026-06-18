#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping


COMPARE_FIELDS = (
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
    "raw_range_m",
    "measured_disparity_px",
    "left_u_px",
    "left_v_px",
    "corrected_right_m",
    "corrected_down_m",
    "corrected_forward_m",
    "corrected_range_m",
    "corrected_public_forward_m",
    "corrected_public_right_m",
    "corrected_public_down_m",
    "corrected_public_range_m",
)


def read_by_group(path: Path) -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    return {str(row.get("group_id", "")): row for row in rows}


def finite_float(value: object) -> float:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


def compare_paths(
    *,
    reference_csv: Path,
    candidate_csvs: Mapping[str, Path],
    tolerance_m: float,
) -> dict[str, object]:
    failures: list[str] = []
    reference = read_by_group(reference_csv)
    require_groups = sorted(reference.keys())
    max_abs_deltas: dict[str, float] = {}
    candidate_row_counts: dict[str, int] = {}
    for candidate_name, candidate_csv in candidate_csvs.items():
        candidate = read_by_group(candidate_csv)
        candidate_row_counts[candidate_name] = len(candidate)
        if sorted(candidate.keys()) != require_groups:
            failures.append(f"{candidate_name}:group_ids_mismatch")
            continue
        for group_id in require_groups:
            ref_row = reference[group_id]
            cand_row = candidate[group_id]
            for field in COMPARE_FIELDS:
                ref_value = finite_float(ref_row.get(field))
                cand_value = finite_float(cand_row.get(field))
                if not math.isfinite(ref_value) or not math.isfinite(cand_value):
                    failures.append(f"{candidate_name}:{group_id}:{field}:not_finite")
                    continue
                delta = abs(cand_value - ref_value)
                key = f"{candidate_name}:{field}"
                max_abs_deltas[key] = max(max_abs_deltas.get(key, 0.0), delta)
                if delta > tolerance_m:
                    failures.append(f"{candidate_name}:{group_id}:{field}:delta_{delta:.12g}_gt_{tolerance_m:.12g}")
    return {
        "status": "PASS" if not failures else "FAIL",
        "reference_csv": str(reference_csv),
        "candidate_csvs": {name: str(path) for name, path in candidate_csvs.items()},
        "reference_row_count": len(reference),
        "candidate_row_counts": candidate_row_counts,
        "tolerance_m": tolerance_m,
        "max_abs_deltas": max_abs_deltas,
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def write_text(path: Path, result: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion path equivalence check",
        f"status: {result['status']}",
        f"reference_csv: {result['reference_csv']}",
        f"reference_row_count: {result['reference_row_count']}",
        f"tolerance_m: {result['tolerance_m']}",
        f"failure_count: {result['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: all runtime paths must produce equivalent internal RDF and public FRD+range outputs.",
    ]
    for name, path_value in dict(result.get("candidate_csvs", {})).items():
        lines.append(f"candidate_csv[{name}]: {path_value}")
    for key, delta in sorted(dict(result.get("max_abs_deltas", {})).items()):
        lines.append(f"max_abs_delta[{key}]: {delta}")
    for failure in result.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check equivalence across observation-fusion runtime input paths.")
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--candidate", action="append", default=[], help="name=path")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--tolerance-m", type=float, default=1.0e-9)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    candidates: dict[str, Path] = {}
    for item in args.candidate:
        if "=" not in item:
            raise SystemExit(f"--candidate must be name=path, got: {item}")
        name, path_value = item.split("=", 1)
        candidates[name] = Path(path_value)
    result = compare_paths(
        reference_csv=args.reference_csv,
        candidate_csvs=candidates,
        tolerance_m=args.tolerance_m,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "observation_fusion_path_equivalence.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(args.output_dir / "observation_fusion_path_equivalence.txt", result)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    if result["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()

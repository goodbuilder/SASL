#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable, Sequence


RDF_AXES = ("right", "down", "forward")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Gate ROI silhouette axis-gain candidates before calling them stable. "
            "This intentionally fails weak evidence such as single-seed reports, "
            "partial inputs, sparse ROI observations, incomplete screen coverage, "
            "or fragile bootstrap subgroups."
        )
    )
    parser.add_argument("--aggregate-summary", type=Path, required=True)
    parser.add_argument("--skipped-inputs", type=Path)
    parser.add_argument("--candidate-mode")
    parser.add_argument("--right-gain", type=float)
    parser.add_argument("--down-gain", type=float)
    parser.add_argument("--forward-gain", type=float)
    parser.add_argument("--candidate-sample-diagnostics", type=Path)
    parser.add_argument("--bootstrap-summary", type=Path)
    parser.add_argument("--profile-coverage-summary", type=Path, action="append", default=[])
    parser.add_argument("--trajectory-diagnostics", type=Path, action="append", default=[])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-completed-reports", type=int, default=6)
    parser.add_argument("--max-skipped-inputs", type=int, default=0)
    parser.add_argument("--min-right-improvement", type=float, default=0.0)
    parser.add_argument("--min-down-improvement", type=float, default=0.0)
    parser.add_argument("--min-forward-improvement", type=float, default=0.0)
    parser.add_argument("--min-valid-roi-fraction", type=float, default=0.20)
    parser.add_argument("--min-validation-trials", type=int, default=5)
    parser.add_argument("--min-validation-screen-bins", type=int, default=9)
    parser.add_argument("--min-valid-roi-per-screen-bin", type=int, default=10)
    parser.add_argument("--min-bootstrap-pass-fraction", type=float, default=0.95)
    parser.add_argument("--bootstrap-groups", default="all,trial,range_bin,screen_bin")
    parser.add_argument("--allow-negative-bootstrap-p025", action="store_true")
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def safe_float(value: object, default: float = float("nan")) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value: object, default: int = 0) -> int:
    try:
        return int(float(str(value)))
    except (TypeError, ValueError):
        return default


def gain_matches(row: dict[str, str], name: str, expected: float | None) -> bool:
    if expected is None:
        return True
    actual = safe_float(row.get(name), default=float("nan"))
    return math.isfinite(actual) and math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9)


def candidate_matches(row: dict[str, str], args: argparse.Namespace) -> bool:
    if args.candidate_mode is not None and row.get("mode") != args.candidate_mode:
        return False
    return (
        gain_matches(row, "right_gain", args.right_gain)
        and gain_matches(row, "down_gain", args.down_gain)
        and gain_matches(row, "forward_gain", args.forward_gain)
    )


def add_check(
    checks: list[dict[str, object]],
    name: str,
    passed: bool,
    *,
    value: object = "",
    threshold: object = "",
    detail: str = "",
) -> None:
    checks.append(
        {
            "check": name,
            "status": "pass" if passed else "fail",
            "value": value,
            "threshold": threshold,
            "detail": detail,
        }
    )


def add_skip(checks: list[dict[str, object]], name: str, *, detail: str) -> None:
    checks.append({"check": name, "status": "skip", "value": "", "threshold": "", "detail": detail})


def count_skipped(path: Path | None) -> int:
    if path is None:
        return 0
    return len(read_csv(path))


def selected_candidate(rows: Sequence[dict[str, str]], args: argparse.Namespace) -> dict[str, str] | None:
    matches = [row for row in rows if candidate_matches(row, args)]
    if matches:
        return matches[0]
    return None if any(
        value is not None
        for value in (args.candidate_mode, args.right_gain, args.down_gain, args.forward_gain)
    ) else (rows[0] if rows else None)


def check_aggregate(args: argparse.Namespace, checks: list[dict[str, object]]) -> dict[str, str] | None:
    rows = read_csv(args.aggregate_summary)
    add_check(
        checks,
        "aggregate_summary_exists",
        bool(rows),
        value=len(rows),
        threshold=">0 rows",
        detail=str(args.aggregate_summary),
    )
    if not rows:
        return None

    candidate = selected_candidate(rows, args)
    add_check(
        checks,
        "candidate_present",
        candidate is not None,
        value=(
            f"{args.candidate_mode},{args.right_gain},{args.down_gain},{args.forward_gain}"
            if any(value is not None for value in (args.candidate_mode, args.right_gain, args.down_gain, args.forward_gain))
            else "first aggregate row"
        ),
        threshold="candidate row",
    )
    if candidate is None:
        return None

    completed_reports = safe_int(candidate.get("required_report_count") or candidate.get("report_count"))
    add_check(
        checks,
        "completed_report_count",
        completed_reports >= args.min_completed_reports,
        value=completed_reports,
        threshold=f">={args.min_completed_reports}",
        detail="loaded reports only; skipped or partial inputs do not count",
    )
    add_check(
        checks,
        "aggregate_pass_all_reports",
        candidate.get("pass_all_reports") == "yes",
        value=candidate.get("pass_all_reports", ""),
        threshold="yes",
    )

    thresholds = {
        "right": args.min_right_improvement,
        "down": args.min_down_improvement,
        "forward": args.min_forward_improvement,
    }
    for axis in RDF_AXES:
        value = safe_float(candidate.get(f"min_{axis}_rmse_improvement_m"))
        add_check(
            checks,
            f"aggregate_min_{axis}_improvement",
            math.isfinite(value) and value >= thresholds[axis],
            value=f"{value:.9f}" if math.isfinite(value) else "nan",
            threshold=f">={thresholds[axis]:.9f}",
        )
    return candidate


def check_skipped_inputs(args: argparse.Namespace, checks: list[dict[str, object]]) -> None:
    skipped_path = args.skipped_inputs
    if skipped_path is None:
        sibling = args.aggregate_summary.parent / "skipped_inputs.csv"
        skipped_path = sibling if sibling.exists() else None
    skipped_count = count_skipped(skipped_path)
    add_check(
        checks,
        "skipped_input_count",
        skipped_count <= args.max_skipped_inputs,
        value=skipped_count,
        threshold=f"<={args.max_skipped_inputs}",
        detail=str(skipped_path) if skipped_path is not None else "no skipped input file",
    )


def unique_values(rows: Iterable[dict[str, str]], key: str) -> set[str]:
    return {row.get(key, "") for row in rows if row.get(key, "") != ""}


def check_sample_diagnostics(args: argparse.Namespace, checks: list[dict[str, object]]) -> None:
    if args.candidate_sample_diagnostics is None:
        add_skip(checks, "candidate_sample_diagnostics", detail="not provided")
        return
    rows = read_csv(args.candidate_sample_diagnostics)
    add_check(
        checks,
        "candidate_sample_diagnostics_exists",
        bool(rows),
        value=len(rows),
        threshold=">0 rows",
        detail=str(args.candidate_sample_diagnostics),
    )
    if not rows:
        return

    valid_rows = [row for row in rows if row.get("fallback") == "none"]
    valid_fraction = len(valid_rows) / max(1, len(rows))
    add_check(
        checks,
        "valid_roi_fraction",
        valid_fraction >= args.min_valid_roi_fraction,
        value=f"{valid_fraction:.6f}",
        threshold=f">={args.min_valid_roi_fraction:.6f}",
        detail=f"{len(valid_rows)} valid / {len(rows)} samples",
    )

    trials = unique_values(rows, "trial")
    add_check(
        checks,
        "validation_trial_count",
        len(trials) >= args.min_validation_trials,
        value=len(trials),
        threshold=f">={args.min_validation_trials}",
    )

    screen_bins = {
        (row.get("screen_u_bin", ""), row.get("screen_v_bin", ""))
        for row in rows
        if row.get("screen_u_bin", "") != "" and row.get("screen_v_bin", "") != ""
    }
    add_check(
        checks,
        "validation_screen_bin_count",
        len(screen_bins) >= args.min_validation_screen_bins,
        value=len(screen_bins),
        threshold=f">={args.min_validation_screen_bins}",
        detail="screen bins observed in candidate validation samples",
    )

    valid_by_screen: dict[tuple[str, str], int] = {}
    for row in valid_rows:
        key = (row.get("screen_u_bin", ""), row.get("screen_v_bin", ""))
        if key[0] == "" or key[1] == "":
            continue
        valid_by_screen[key] = valid_by_screen.get(key, 0) + 1
    min_valid_per_screen = min(valid_by_screen.values()) if valid_by_screen else 0
    add_check(
        checks,
        "min_valid_roi_per_screen_bin",
        len(screen_bins) >= args.min_validation_screen_bins
        and min_valid_per_screen >= args.min_valid_roi_per_screen_bin,
        value=min_valid_per_screen,
        threshold=f">={args.min_valid_roi_per_screen_bin}",
        detail=f"populated valid screen bins={len(valid_by_screen)}",
    )


def bootstrap_row_label(row: dict[str, str]) -> str:
    group = row.get("group", "")
    if group == "trial":
        return f"trial={row.get('trial', '')}"
    if group == "range_bin":
        return f"range_bin={row.get('range_bin', '')}"
    if group == "screen_bin":
        return f"screen_bin={row.get('screen_u_bin', '')}:{row.get('screen_v_bin', '')}"
    return group or "unknown"


def check_bootstrap(args: argparse.Namespace, checks: list[dict[str, object]]) -> None:
    if args.bootstrap_summary is None:
        add_skip(checks, "bootstrap_summary", detail="not provided")
        return
    rows = read_csv(args.bootstrap_summary)
    add_check(
        checks,
        "bootstrap_summary_exists",
        bool(rows),
        value=len(rows),
        threshold=">0 rows",
        detail=str(args.bootstrap_summary),
    )
    if not rows:
        return
    required_groups = {part.strip() for part in args.bootstrap_groups.split(",") if part.strip()}
    scoped_rows = [row for row in rows if row.get("group", "") in required_groups]
    add_check(
        checks,
        "bootstrap_required_groups_present",
        bool(scoped_rows),
        value=len(scoped_rows),
        threshold=f"groups={','.join(sorted(required_groups))}",
    )
    for row in scoped_rows:
        label = bootstrap_row_label(row)
        pass_fraction = safe_float(row.get("bootstrap_pass_fraction"))
        add_check(
            checks,
            f"bootstrap_pass_fraction:{label}",
            math.isfinite(pass_fraction) and pass_fraction >= args.min_bootstrap_pass_fraction,
            value=f"{pass_fraction:.9f}" if math.isfinite(pass_fraction) else "nan",
            threshold=f">={args.min_bootstrap_pass_fraction:.9f}",
        )
        if args.allow_negative_bootstrap_p025:
            continue
        for axis in RDF_AXES:
            p025 = safe_float(row.get(f"{axis}_p025_m"))
            add_check(
                checks,
                f"bootstrap_{axis}_p025:{label}",
                math.isfinite(p025) and p025 >= 0.0,
                value=f"{p025:.9f}" if math.isfinite(p025) else "nan",
                threshold=">=0.000000000",
            )


def check_profile_coverage(args: argparse.Namespace, checks: list[dict[str, object]]) -> None:
    if not args.profile_coverage_summary:
        add_skip(checks, "profile_coverage_summary", detail="not provided")
        return
    for path in args.profile_coverage_summary:
        if not path.exists():
            add_check(
                checks,
                f"profile_coverage_summary_exists:{path}",
                False,
                value="missing",
                threshold="file exists",
            )
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        count = safe_int(data.get("screen_bin_coverage_count"))
        total = safe_int(data.get("screen_bin_total"), default=args.min_validation_screen_bins)
        add_check(
            checks,
            f"profile_screen_bin_coverage:{path}",
            total > 0 and count >= total,
            value=f"{count}/{total}",
            threshold="all profile screen bins",
        )


def check_trajectory_diagnostics(args: argparse.Namespace, checks: list[dict[str, object]]) -> None:
    if not args.trajectory_diagnostics:
        add_skip(checks, "trajectory_diagnostics", detail="not provided")
        return
    for path in args.trajectory_diagnostics:
        if not path.exists():
            add_check(
                checks,
                f"trajectory_diagnostics_exists:{path}",
                False,
                value="missing",
                threshold="file exists",
            )
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        count = safe_int(data.get("validation_screen_bin_count"))
        total = safe_int(data.get("screen_bin_total"), default=args.min_validation_screen_bins)
        add_check(
            checks,
            f"trajectory_validation_screen_coverage:{path}",
            total > 0 and count >= min(total, args.min_validation_screen_bins),
            value=f"{count}/{total}",
            threshold=f">={min(total, args.min_validation_screen_bins)} bins",
        )


def main() -> int:
    args = parse_args()
    checks: list[dict[str, object]] = []
    candidate = check_aggregate(args, checks)
    check_skipped_inputs(args, checks)
    check_sample_diagnostics(args, checks)
    check_bootstrap(args, checks)
    check_profile_coverage(args, checks)
    check_trajectory_diagnostics(args, checks)

    failed = [row for row in checks if row["status"] == "fail"]
    result = {
        "status": "pass" if not failed else "fail",
        "failed_check_count": len(failed),
        "check_count": len(checks),
        "candidate": candidate or {},
        "failed_checks": failed,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "stability_gate_checks.csv", checks)
    (args.output_dir / "stability_gate_summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"Status: {result['status']}")
    print(f"Checks: {len(checks)} total, {len(failed)} failed")
    print(f"Summary: {args.output_dir / 'stability_gate_summary.json'}")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())

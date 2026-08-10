#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_stereodistance_operator_view_launch as operator_checker  # noqa: E402


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check operator-view dry-run readiness for every active profile produced by "
            "run_runtime_guard_operating_mode_batch_smoke.py."
        )
    )
    parser.add_argument("--batch-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime guard batch operator-view readiness",
        f"status: {summary.get('status', '')}",
        f"batch_csv: {summary.get('batch_csv', '')}",
        f"run_count: {summary.get('run_count', 0)}",
        f"pass_count: {summary.get('pass_count', 0)}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', False)}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"failure_count: {summary.get('failure_count', 0)}",
        "",
        "Rows:",
    ]
    for row in rows:
        lines.append(
            "- {scenario}/{seed}: status={status} package={package_type} profile={profile_id} resolved={resolved}".format(
                scenario=row.get("scenario", ""),
                seed=row.get("seed", ""),
                status=row.get("status", ""),
                package_type=row.get("package_type", ""),
                profile_id=row.get("profile_id", ""),
                resolved=row.get("resolved_runtime_profile", ""),
            )
        )
        failures = str(row.get("failures", "")).strip()
        if failures:
            lines.append(f"  failures={failures}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def row_output_dir(output_dir: Path, row: Mapping[str, object], index: int) -> Path:
    scenario = str(row.get("scenario", "") or f"row{index}")
    seed = str(row.get("seed", "") or "seed")
    return output_dir / scenario / seed


def check_row(row: Mapping[str, str], *, output_dir: Path, index: int) -> dict[str, object]:
    scenario = str(row.get("scenario", ""))
    seed = str(row.get("seed", ""))
    active_profile = Path(str(row.get("active_profile", "")))
    package_dir = active_profile.parent
    run_dir = row_output_dir(output_dir, row, index)
    operator_json = run_dir / "operator_view_launch_check.json"
    operator_txt = run_dir / "operator_view_launch_check.txt"
    failures: list[str] = []
    if not active_profile.is_file():
        failures.append("active_profile:missing")
    if not package_dir.is_dir():
        failures.append("package_dir:missing")
    operator_result: dict[str, object] = {}
    if not failures:
        operator_result = operator_checker.check_operator_view_launch(package_dir=package_dir)
        operator_checker.write_json(operator_json, operator_result)
        operator_checker.write_text(operator_txt, operator_result)
        if operator_result.get("status") != "PASS":
            failures.append("operator_view_check_failed")
        if operator_result.get("resolved_runtime_profile") != str(active_profile.resolve()):
            failures.append("operator_view_resolved_profile_mismatch")
        if bool(operator_result.get("runtime_truth_used", True)):
            failures.append("operator_view_runtime_truth_used")
    status = "PASS" if not failures else "FAIL"
    return {
        "scenario": scenario,
        "seed": seed,
        "status": status,
        "package_dir": str(package_dir),
        "active_profile": str(active_profile),
        "operator_json": str(operator_json) if operator_result else "",
        "operator_txt": str(operator_txt) if operator_result else "",
        "source_batch_status": row.get("status", ""),
        "package_type": operator_result.get("package_type", ""),
        "package_check_status": operator_result.get("package_check_status", ""),
        "operator_returncode": operator_result.get("operator_command_returncode", ""),
        "launch_contract_status": operator_result.get("launch_contract_status", ""),
        "resolved_runtime_profile": operator_result.get("resolved_runtime_profile", ""),
        "profile_id": operator_result.get("profile_id", ""),
        "method": operator_result.get("method", ""),
        "profile_mode": operator_result.get("profile_mode", ""),
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "failures": ",".join(failures),
    }


def check_batch(*, batch_csv: Path, output_dir: Path) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    input_rows = read_csv(batch_csv)
    rows = [check_row(row, output_dir=output_dir, index=index) for index, row in enumerate(input_rows)]
    pass_count = sum(1 for row in rows if row.get("status") == "PASS")
    failures = []
    if not rows:
        failures.append("batch_csv:no_rows")
    failures.extend(
        f"{row.get('scenario')}/{row.get('seed')}:{row.get('failures')}"
        for row in rows
        if row.get("status") != "PASS"
    )
    summary = {
        "mode": "runtime_guard_batch_operator_view_readiness",
        "status": "PASS" if rows and pass_count == len(rows) else "FAIL",
        "batch_csv": str(batch_csv),
        "output_dir": str(output_dir),
        "run_count": len(rows),
        "pass_count": pass_count,
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": (
            "operator wrapper receives lightweight runtime_guard profile packages whose active profiles "
            "consume stereo-center RDF(right,down,forward) plus measured disparity"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }
    write_csv(output_dir / "runtime_guard_batch_operator_view_readiness.csv", rows)
    write_json(output_dir / "runtime_guard_batch_operator_view_readiness_summary.json", summary)
    write_text(output_dir / "runtime_guard_batch_operator_view_readiness_summary.txt", summary, rows)
    return summary


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return check_batch(batch_csv=args.batch_csv, output_dir=args.output_dir)


def main() -> None:
    summary = run()
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
